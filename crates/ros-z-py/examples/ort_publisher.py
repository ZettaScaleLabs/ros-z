#!/usr/bin/env python3
"""
ort_publisher.py — Preprocess an image on the GPU and publish it as a typed
CUDA tensor via ros-z-py for ort_classifier.rs to run SqueezeNet inference on.

Requirements:
    - ros_z_py built with --features cuda
    - CUDA-capable GPU
    - torch, torchvision, Pillow

Usage:
    # Terminal 1 (start subscriber first):
    cargo run --example ort_classifier --features jazzy,cuda

    # Terminal 2:
    python crates/ros-z-py/examples/ort_publisher.py [path/to/image.jpg]

Default image: assets/test_image.jpg
"""

import sys
import time

try:
    import torch
    import torchvision.transforms.v2 as T
    from PIL import Image
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install: pip install torch torchvision pillow")
    sys.exit(1)

try:
    from ros_z_py import ZContextBuilder, sensor_msgs
except ImportError as e:
    print(f"Failed to import ros_z_py (build with --features cuda): {e}")
    sys.exit(1)

TOPIC = "/ort/image"
DEVICE_ID = 0

# ImageNet normalization constants
MEAN = [0.485, 0.456, 0.406]
STD = [0.229, 0.224, 0.225]

# SqueezeNet 1.1 input: [1, 3, 224, 224] f32 NCHW
transform = T.Compose(
    [
        T.Resize(256),
        T.CenterCrop(224),
        T.ToImage(),
        T.ToDtype(torch.float32, scale=True),
        T.Normalize(mean=MEAN, std=STD),
    ]
)

# Direct peer connection: subscriber listens, publisher connects.
SUB_LISTEN = "tcp/127.0.0.1:7449"


def main() -> None:
    image_path = sys.argv[1] if len(sys.argv) > 1 else "assets/test_image.jpg"

    print(f"[pub] loading image: {image_path}")
    img = Image.open(image_path).convert("RGB")

    tensor = (
        transform(img).unsqueeze(0).cuda(DEVICE_ID).contiguous()
    )  # [1, 3, 224, 224] f32 NCHW
    torch.cuda.synchronize(DEVICE_ID)
    print(
        f"[pub] tensor ready: shape={list(tensor.shape)} "
        f"dtype={tensor.dtype} device={tensor.device} "
        f"nbytes={tensor.nbytes}"
    )

    ctx = (
        ZContextBuilder()
        .with_shm_enabled()
        .with_connect_endpoints([SUB_LISTEN])
        .build()
    )
    node = ctx.create_node("ort_publisher").build()
    pub = node.create_publisher(TOPIC, sensor_msgs.LaserScan)

    # Wait for subscriber to declare before first publish.
    print("[pub] waiting for subscriber …")
    time.sleep(2.0)

    print(f"[pub] publishing on {TOPIC} at 1 Hz")
    while True:
        pub.publish_tensor(tensor)
        print("[pub] published")
        time.sleep(1.0)


if __name__ == "__main__":
    main()
