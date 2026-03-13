//! ort_classifier — receive a CUDA tensor from ort_publisher.py, run SqueezeNet 1.1 via ONNX
//! Runtime's CUDAExecutionProvider, and print the top-1 ImageNet label.
//!
//! # Setup
//!
//! 1. Download the model (first time only):
//!    ```bash
//!    curl -L https://huggingface.co/onnxmodelzoo/squeezenet1.1-7/resolve/main/squeezenet1.1-7.onnx \
//!         -o assets/squeezenet1.1-7.onnx
//!    ```
//!
//! 2. Start the Rust subscriber first, then the Python publisher:
//!    ```bash
//!    # Terminal 1
//!    CUDA_PATH=/nix/store/addcyrkwfz2ghah3vrqvda547xafpgsa-cuda_cudart-12.8.90-lib \
//!      cargo run --example ort_classifier --features jazzy,cuda
//!
//!    # Terminal 2
//!    python crates/ros-z-py/examples/ort_publisher.py assets/test_image.jpg
//!    ```
//!
//! # Data path
//!
//! ```text
//! Python (torch GPU tensor)
//!   → publish_tensor()           IPC handle on wire (no CPU copy)
//!   → recv_serialized()          raw zenoh Sample, CUDA ZSlice preserved
//!   → copy_to_host()             D2H: [1,3,224,224] f32 (one copy for ort input)
//!   → ort CUDA session           re-uploaded by CUDAExecutionProvider
//!   → softmaxout_1 argmax        top-1 class label
//! ```
//!
//! # Note on ORT native library
//!
//! `ort` downloads ONNX Runtime automatically (ORT_STRATEGY=download).
//! In a Nix environment set `ORT_LIB_LOCATION` to the system ONNX Runtime path.

use std::time::Duration;

use ndarray::Array4;
use ort::{
    execution_providers::CUDAExecutionProvider,
    session::{Session, builder::GraphOptimizationLevel},
    value::TensorRef,
};
use ros_z::{Builder, ZBuf, context::ZContextBuilder};
use ros_z_msgs::sensor_msgs::LaserScan;
use serde_json::json;
use zenoh_buffers::ZBuf as ZenohZBuf;

const TOPIC: &str = "/ort/image";
const MODEL: &str = "assets/squeezenet1.1-7.onnx";
const LABELS: &str = "assets/imagenet_classes.txt";

/// Direct peer: Python publisher connects to us on this address.
const LISTEN_ADDR: &str = "tcp/127.0.0.1:7449";

fn load_labels(path: &str) -> Result<Vec<String>, std::io::Error> {
    let content = std::fs::read_to_string(path)?;
    Ok(content.lines().map(String::from).collect())
}

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    zenoh::init_log_from_env_or("warn");

    // --- ORT session on GPU 0 ---
    // ort::Error contains raw pointers (not Send+Sync) so we use .expect() here.
    let mut session = Session::builder()
        .expect("ort SessionBuilder")
        .with_execution_providers([CUDAExecutionProvider::default().build()])
        .expect("ort CUDA EP")
        .with_optimization_level(GraphOptimizationLevel::Level3)
        .expect("ort optimization level")
        .commit_from_file(MODEL)
        .expect("ort commit_from_file");

    let labels = load_labels(LABELS)?;
    println!("loaded {} labels, model ready", labels.len());

    // --- ros-z subscriber (direct peer, publisher connects to us) ---
    let ctx = ZContextBuilder::default()
        .with_json("listen/endpoints", json!([LISTEN_ADDR]))
        .with_shm_enabled()?
        .build()?;
    let node = ctx.create_node("ort_classifier").build()?;
    // recv_serialized_timeout() bypasses CDR — CUDA ZSlice kind bits are preserved.
    let sub = node.create_sub::<LaserScan>(TOPIC).build()?;

    println!("listening on {TOPIC} (peer addr {LISTEN_ADDR})");
    println!("run: python crates/ros-z-py/examples/ort_publisher.py [image.jpg]");

    loop {
        let sample = match sub.recv_serialized_timeout(Duration::from_secs(60)) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("recv timeout: {e}");
                continue;
            }
        };

        // Convert zenoh payload → ZBuf (preserves CudaTensor ZSlice kind)
        let zenoh_zbuf: ZenohZBuf = sample.payload().clone().into();
        let zbuf = ZBuf::from_zenoh(zenoh_zbuf);

        let Some((cuda_buf, meta)) = zbuf.typed_cuda_slices().next() else {
            eprintln!("received non-CUDA-tensor message, skipping");
            continue;
        };
        // Validate: [1, 3, 224, 224] f32
        if meta.shape.len() != 4 {
            eprintln!(
                "unexpected ndim {} (expected 4), skipping",
                meta.shape.len()
            );
            continue;
        }
        if meta.dtype_bits != 32 || meta.dtype_code != 2 {
            eprintln!(
                "unexpected dtype bits={} code={} (expected f32), skipping",
                meta.dtype_bits, meta.dtype_code
            );
            continue;
        }

        let shape: [usize; 4] = [
            meta.shape[0] as usize,
            meta.shape[1] as usize,
            meta.shape[2] as usize,
            meta.shape[3] as usize,
        ];
        let n_elems: usize = shape.iter().product();
        let n_bytes = n_elems * 4; // f32

        // Approach B: device → host (one D2H transfer per frame)
        let mut host_buf = vec![0u8; n_bytes];
        cuda_buf.copy_to_host(&mut host_buf)?;

        // Reinterpret as little-endian f32
        let floats: Vec<f32> = host_buf
            .chunks_exact(4)
            .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .collect();

        // Build ndarray [n, c, h, w] and run inference
        let arr = Array4::from_shape_vec(shape, floats)?;
        let inputs = ort::inputs![TensorRef::from_array_view(arr.view()).expect("tensor ref")];
        let outputs = session.run(inputs).expect("ort run");

        // SqueezeNet output: first output, shape [1, 1000] or [1, 1000, 1, 1]
        let (_, probs) = outputs[0]
            .try_extract_tensor::<f32>()
            .expect("extract tensor");

        let (top1_idx, &top1_score) = probs
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap();

        let label = labels
            .get(top1_idx)
            .map(String::as_str)
            .unwrap_or("unknown");
        println!("top-1: {} ({:.1}%)", label, top1_score * 100.0);
    }
}
