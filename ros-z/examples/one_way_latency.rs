use std::{
    io::Write,
    rc::Rc,
    str::FromStr,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use zenoh::{Result, pubsub::Publisher, qos::CongestionControl};

const TOKIO_SERVER_ADDR: &str = "127.0.0.1:12345";
const GLOMMIO_SERVER_ADDR: &str = "127.0.0.1:12346";
const ZENOH_KEYEXPR: &str = "one_way_latency";

use clap::Parser;
#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "64")]
    payload: usize,
    #[arg(short, long, default_value = "10")]
    frequency: usize,
    #[arg(short, long, default_value = "100")]
    sample: usize,
    #[arg(short, long, default_value = "")]
    config: String,

    // For Zenoh TX
    #[arg(long)]
    express: bool,
    #[arg(long)]
    cc_block: bool,

    // For TX
    #[arg(long, default_value = "zenoh")]
    mode: Mode,
    #[arg(long)]
    recv: bool,

    // For RX
    #[arg(long, default_value = "")]
    log: String,
    #[arg(long, default_value = "2")]
    warmup: usize,
}

#[derive(Debug, Clone, Copy, Default)]
enum Mode {
    Tokio,
    Udp,
    Foo,
    #[default]
    Zenoh,
    Glommio,
    Monoio,
    StdTcp,
    StdUdp,
}

impl FromStr for Mode {
    type Err = String;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s {
            "tokio" | "Tokio" => Ok(Mode::Tokio),
            "udp" | "Udp" => Ok(Mode::Udp),
            "foo" | "Foo" => Ok(Mode::Foo),
            "zenoh" | "Zenoh" => Ok(Mode::Zenoh),
            "glommio" | "Glommio" => Ok(Mode::Glommio),
            "monoio" | "Monoio" => Ok(Mode::Monoio),
            "std_tcp" | "StdTcp" => Ok(Mode::StdTcp),
            "std_udp" | "StdUdp" => Ok(Mode::StdUdp),
            _ => Err(format!("This mode {s} is not supported.")),
        }
    }
}

type Timestamp = u64;

mod sync {
    use crate::rx::{DataLogger, print_statistics};
    use crate::{Args, TOKIO_SERVER_ADDR, Timestamp};
    use std::io::Write;
    use std::net::{TcpListener, TcpStream, UdpSocket};
    use std::path::PathBuf;
    use std::rc::Rc;
    use std::sync::Arc;
    use std::time::{SystemTime, UNIX_EPOCH};
    use zenoh::Result;

    #[derive(Default)]
    pub struct Notifier {
        pub mutex: parking_lot::Mutex<()>,
        pub cv: parking_lot::Condvar,
    }

    impl Notifier {
        pub fn notify_all(&self) {
            let _ = self.mutex.lock();
            self.cv.notify_all();
        }
    }

    pub trait Sendable: Sized {
        fn init(args: Rc<Args>) -> Result<Self>;
        fn send(&mut self, msg: &[u8]) -> Result<()>;
    }

    pub trait Receivable: Sized {
        fn init(args: Rc<Args>) -> Result<Self>;
        fn recv(&mut self) -> Result<Timestamp>;
    }

    pub struct StdTcpSender {
        stream: TcpStream,
    }

    impl Sendable for StdTcpSender {
        fn init(_args: Rc<Args>) -> Result<Self> {
            let stream = TcpStream::connect(TOKIO_SERVER_ADDR)?;
            stream.set_nodelay(true)?;
            Ok(Self { stream })
        }

        fn send(&mut self, msg: &[u8]) -> Result<()> {
            use std::io::Write;
            self.stream.write_all(msg)?;
            Ok(())
        }
    }

    pub struct StdUdpSender {
        socket: UdpSocket,
    }

    impl Sendable for StdUdpSender {
        fn init(_args: Rc<Args>) -> Result<Self> {
            let socket = UdpSocket::bind("127.0.0.1:0")?;
            socket.connect(TOKIO_SERVER_ADDR)?;
            Ok(Self { socket })
        }

        fn send(&mut self, msg: &[u8]) -> Result<()> {
            self.socket.send(msg)?;
            Ok(())
        }
    }

    pub struct StdTcpReceiver {
        // queue: flume::Receiver<Vec<u8>>,
        queue: Arc<crossbeam_queue::ArrayQueue<Vec<u8>>>,
        _task: std::thread::JoinHandle<()>,
        notifier: Arc<Notifier>,
    }

    impl Receivable for StdTcpReceiver {
        fn init(args: Rc<Args>) -> Result<Self> {
            use std::io::Read;
            let listener = TcpListener::bind(TOKIO_SERVER_ADDR)?;
            let (mut stream, _) = listener.accept()?;
            let queue = Arc::new(crossbeam_queue::ArrayQueue::new(10));
            let c_queue = queue.clone();
            let len = args.payload;
            let notifier = Arc::new(Notifier::default());
            let c_notifier = notifier.clone();
            let task = std::thread::spawn(move || {
                let mut buf = vec![0u8; 63336];
                while let Ok(n) = stream.read(&mut buf) {
                    if n == 0 {
                        continue;
                    }
                    // assert!(n % len == 0);
                    for idx in 0..(n / len) {
                        let i = idx * len;
                        let j = i + len;
                        // tx.send(buf[i..j].to_owned()).unwrap();
                        let _ = queue.force_push(buf[i..j].to_owned());
                        c_notifier.notify_all();
                    }
                }
            });
            Ok(Self {
                queue: c_queue,
                _task: task,
                notifier,
            })
        }

        fn recv(&mut self) -> Result<Timestamp> {
            if self.queue.is_empty() {
                let (mutex, cv) = {
                    let x = self.notifier.as_ref();
                    (&x.mutex, &x.cv)
                };
                let mut lock = mutex.lock();
                cv.wait_while(&mut lock, |_| self.queue.is_empty());
            }
            let msg = self.queue.pop().unwrap();
            let x = msg[0..size_of::<Timestamp>()].try_into()?;
            Ok(Timestamp::from_le_bytes(x))
        }
    }

    pub struct StdUdpReceiver {
        // queue: flume::Receiver<Vec<u8>>,
        queue: Arc<crossbeam_queue::ArrayQueue<Vec<u8>>>,
        _task: std::thread::JoinHandle<()>,
        notifier: Arc<Notifier>,
    }

    impl Receivable for StdUdpReceiver {
        fn init(args: Rc<Args>) -> Result<Self> {
            let socket = UdpSocket::bind(TOKIO_SERVER_ADDR)?;
            let queue = Arc::new(crossbeam_queue::ArrayQueue::new(10));
            let c_queue = queue.clone();
            let len = args.payload;
            let notifier = Arc::new(Notifier::default());
            let c_notifier = notifier.clone();
            let task = std::thread::spawn(move || {
                let mut buf = vec![0u8; len];
                while let Ok(n) = socket.recv(&mut buf) {
                    assert!(n == len);
                    let _ = queue.force_push(buf.to_vec());
                    c_notifier.notify_all();
                }
            });
            Ok(Self {
                queue: c_queue,
                _task: task,
                notifier,
            })
        }

        fn recv(&mut self) -> Result<Timestamp> {
            if self.queue.is_empty() {
                let (mutex, cv) = {
                    let x = self.notifier.as_ref();
                    (&x.mutex, &x.cv)
                };
                let mut lock = mutex.lock();
                cv.wait_while(&mut lock, |_| self.queue.is_empty());
            }
            let msg = self.queue.pop().unwrap();
            let x = msg[0..size_of::<Timestamp>()].try_into()?;
            Ok(Timestamp::from_le_bytes(x))
        }
    }

    pub fn tx_task<S: Sendable>(args: Rc<Args>) -> Result<()> {
        tracing::info!("Std Sender");

        let mut sender = S::init(args.clone())?;

        let period = if args.frequency == 0 {
            None
        } else {
            Some(std::time::Duration::from_secs_f64(
                1.0 / args.frequency as f64,
            ))
        };

        let mut msg = vec![0xAA; args.payload];
        let mut idx = 0;
        loop {
            if let Some(period) = period {
                std::thread::sleep(period);
            }

            let now_timestamp =
                SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as Timestamp;
            msg[0..size_of::<Timestamp>()].copy_from_slice(&now_timestamp.to_le_bytes());
            sender.send(&msg)?;

            idx += 1;
            if args.sample > 0 && idx > args.sample {
                break Ok(());
            }
        }
    }

    pub fn rx_task<R: Receivable>(args: Rc<Args>) -> Result<()> {
        tracing::info!("Std Receiver");

        let mut receiver = R::init(args.clone())?;

        tracing::info!("Begining the warm up...");
        let now = std::time::Instant::now();
        let warmup = std::time::Duration::from_secs(args.warmup as u64);
        let warmup_samples = (args.sample as f64 * 0.2) as usize;
        std::io::stdout().flush().unwrap();
        let mut counter = 0;
        while now.elapsed() < warmup || counter <= warmup_samples {
            receiver.recv()?;
            counter += 1;
        }
        tracing::info!("Done.");

        let mut lats = Vec::with_capacity(args.sample);

        while let Ok(sent_time) = receiver.recv() {
            let now_ns = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as u64;
            let lat = now_ns - sent_time;
            if lat == 0 {
                tracing::error!("[rx] lat = 0");
                continue;
            }
            lats.push(lat as _);
            if lats.len() >= args.sample {
                break;
            }
        }
        print_statistics(lats.clone());

        if !args.log.is_empty() {
            let logger = DataLogger {
                frequency: args.frequency,
                payload: args.payload,
                path: PathBuf::from(args.log.clone()),
            };

            logger.write(lats)?;
        }
        Ok(())
    }
}

mod tx {
    use std::rc::Rc;

    use tokio::net::UdpSocket;
    use zenoh::Session;

    use super::*;
    pub trait Sendable: Sized {
        fn async_runtime() -> AsyncRuntime;
        async fn init(args: Rc<Args>) -> Result<Self>;
        async fn send(&mut self, msg: &[u8]) -> Result<()>;
    }

    pub struct ZenohSender {
        _sess: Session,
        zpub: Publisher<'static>,
    }

    impl Sendable for ZenohSender {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Tokio
        }

        async fn init(args: Rc<Args>) -> Result<Self> {
            let config = if args.config.is_empty() {
                zenoh::Config::default()
            } else {
                zenoh::Config::from_file(&*args.config.clone())?
            };
            let session = zenoh::open(config).await?;
            let zpub = session
                .declare_publisher(ZENOH_KEYEXPR)
                .congestion_control(if args.cc_block {
                    CongestionControl::Block
                } else {
                    CongestionControl::Drop
                })
                .express(args.express)
                .await?;

            print!("Waiting for a subscirber...");
            std::io::stdout().flush()?;
            zpub.matching_listener().await?.recv()?;
            println!("done.");

            Ok(Self {
                _sess: session,
                zpub,
            })
        }

        async fn send(&mut self, msg: &[u8]) -> Result<()> {
            self.zpub.put(msg).await?;
            Ok(())
        }
    }

    pub struct TokioSender {
        stream: tokio::net::TcpStream,
    }

    impl Sendable for TokioSender {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Tokio
        }

        async fn init(_args: Rc<Args>) -> Result<Self> {
            let stream = tokio::net::TcpStream::connect(TOKIO_SERVER_ADDR).await?;
            stream.set_nodelay(true)?;
            tracing::info!("Connected to a TcpListener.");
            Ok(Self { stream })
        }

        async fn send(&mut self, msg: &[u8]) -> Result<()> {
            use tokio::io::AsyncWriteExt;
            self.stream.write_all(msg).await?;
            Ok(())
        }
    }

    pub struct TokioUdpSender {
        socket: UdpSocket,
    }

    impl Sendable for TokioUdpSender {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Tokio
        }

        async fn init(_args: Rc<Args>) -> Result<Self> {
            let socket = UdpSocket::bind("127.0.0.1:7447").await?;
            socket.connect(TOKIO_SERVER_ADDR).await?;
            Ok(Self { socket })
        }

        async fn send(&mut self, msg: &[u8]) -> Result<()> {
            self.socket.send(msg).await?;
            Ok(())
        }
    }

    pub struct GlommioSender {
        stream: glommio::net::TcpStream,
    }

    impl Sendable for GlommioSender {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Glommio
        }

        async fn init(_args: Rc<Args>) -> Result<Self> {
            let stream = glommio::net::TcpStream::connect(GLOMMIO_SERVER_ADDR)
                .await
                .unwrap();
            stream.set_nodelay(true).unwrap();
            tracing::info!("Connected to TcpListener.");
            Ok(Self { stream })
        }

        async fn send(&mut self, msg: &[u8]) -> Result<()> {
            use futures::AsyncWriteExt;
            self.stream.write_all(msg).await?;
            Ok(())
        }
    }

    pub struct MonoioSender {
        stream: monoio::net::TcpStream,
    }

    impl Sendable for MonoioSender {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Monoio
        }

        async fn init(_args: Rc<Args>) -> Result<Self> {
            let stream = monoio::net::TcpStream::connect(GLOMMIO_SERVER_ADDR).await?;
            stream.set_nodelay(true).unwrap();
            tracing::info!("Connected to TcpListener.");
            Ok(Self { stream })
        }

        async fn send(&mut self, msg: &[u8]) -> Result<()> {
            use monoio::io::AsyncWriteRentExt;

            let (res, _) = self.stream.write_all(msg.to_vec()).await;
            res?;
            Ok(())
        }
    }

    pub fn tx_task<S: Sendable>(args: Rc<Args>) -> Result<()> {
        let rt = S::async_runtime();
        tracing::info!("Async runtime: {rt:?}");

        enum Timer {
            Tokio(tokio::time::Duration, tokio::time::Instant),
            Glommio(std::time::Duration),
            Monoio(std::time::Duration),
        }

        impl Timer {
            async fn sleep(&mut self) {
                match self {
                    Timer::Tokio(step, instant) => {
                        *instant = *instant + *step;
                        tokio::time::sleep_until(*instant).await;
                    }
                    Timer::Glommio(dur) => {
                        glommio::timer::sleep(*dur).await;
                    }
                    Timer::Monoio(dur) => {
                        monoio::time::sleep(*dur).await;
                    }
                }
            }
        }

        let mut msg = vec![0xAA; args.payload];
        rt.clone().block_on(async move {
            let mut timer = if args.frequency == 0 {
                None
            } else {
                let x = Duration::from_secs_f64(1.0 / args.frequency as f64);
                Some(match rt {
                    AsyncRuntime::Tokio => Timer::Tokio(x.into(), tokio::time::Instant::now()),
                    AsyncRuntime::Monoio => Timer::Monoio(x),
                    AsyncRuntime::Glommio => Timer::Glommio(x),
                })
            };

            let mut sender = S::init(args.clone()).await?;
            let mut now = std::time::Instant::now();
            let mut idx = 0;

            loop {
                if let Some(timer) = &mut timer {
                    timer.sleep().await;
                }

                let now_timestamp =
                    SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as Timestamp;
                msg[0..size_of::<Timestamp>()].copy_from_slice(&now_timestamp.to_le_bytes());
                if sender.send(&msg).await.is_err() {
                    break;
                }

                idx += 1;

                if args.sample != 0 && idx >= args.sample {
                    break;
                }

                if now.elapsed() >= std::time::Duration::from_secs(1) {
                    now = std::time::Instant::now();
                    tracing::debug!("[tx_task] idx = {idx}");
                }
            }
            tracing::info!("Finished, idx = {idx}");
            Result::Ok(())
        })
    }
}

mod rx {

    use glommio::channels::local_channel;
    use glommio::channels::local_channel::LocalReceiver;
    use std::{collections::VecDeque, fs::File, path::PathBuf, sync::Arc};
    use tokio::net::UdpSocket;

    use crossbeam_queue::ArrayQueue;
    use csv::Writer;
    use tokio::io::AsyncReadExt;
    use zenoh::{Session, handlers::FifoChannelHandler, pubsub::Subscriber, sample::Sample};
    // use deadqueue::limited::Queue;
    use tokio::sync::Semaphore;

    use super::*;

    #[derive(Debug)]
    pub struct DataLogger {
        pub payload: usize,
        pub frequency: usize,
        pub path: PathBuf,
    }

    impl DataLogger {
        pub fn write(&self, data: Vec<u64>) -> Result<()> {
            let file = File::create(&self.path)?;
            let mut wtr = Writer::from_writer(file);
            wtr.write_record(
                ["Frequency", "Payload", "Latency"]
                    .iter()
                    .map(|x| x.to_string()),
            )?;

            for val in data {
                wtr.write_record(
                    [self.frequency, self.payload, val as _]
                        .iter()
                        .map(|x| x.to_string()),
                )?;
                wtr.flush()?;
            }
            Ok(())
        }
    }

    pub fn print_statistics(mut data: Vec<Timestamp>) {
        fn get_percentile(data: &[Timestamp], percentile: f64) -> Timestamp {
            if data.is_empty() {
                return 0;
            }
            let idx = ((percentile * data.len() as f64).round() as usize).min(data.len() - 1);
            data[idx]
        }

        data.sort();
        println!("\nLatency stats (nanoseconds):");
        println!("Min : {}", data.first().unwrap());
        println!("p05 : {}", get_percentile(&data, 0.05));
        println!("p50 : {}", get_percentile(&data, 0.50));
        println!("p95 : {}", get_percentile(&data, 0.95));
        println!("Max : {}", data.last().unwrap());
    }

    pub trait Receivable: Sized {
        fn async_runtime() -> AsyncRuntime;
        async fn init(args: Rc<Args>) -> Result<Self>;

        async fn recv(&mut self) -> Result<Timestamp>;

        fn clean(&self) {}
    }

    pub struct ZenohReceiver {
        _sess: Session,
        zsub: Subscriber<FifoChannelHandler<Sample>>,
    }

    impl Receivable for ZenohReceiver {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Tokio
        }

        async fn init(args: Rc<Args>) -> Result<Self> {
            let config = if args.config.is_empty() {
                zenoh::Config::default()
            } else {
                zenoh::Config::from_file(args.config.clone())?
            };
            let session = zenoh::open(config).await?;

            let zsub = session.declare_subscriber(ZENOH_KEYEXPR).await?;

            Ok(Self {
                _sess: session,
                zsub,
            })
        }

        async fn recv(&mut self) -> Result<Timestamp> {
            let sample = self.zsub.recv_async().await?;
            let msg = sample.payload().to_bytes();
            Ok(Timestamp::from_le_bytes(
                msg[0..size_of::<Timestamp>()].try_into()?,
            ))
        }

        // fn clean(&self) {
        //     self.zsub.drain();
        // }
    }

    pub struct FooReceiver {
        _sess: Session,
        queue: Arc<Queue2<Vec<u8>>>,
    }

    impl Receivable for FooReceiver {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Tokio
        }

        async fn init(args: Rc<Args>) -> Result<Self> {
            let config = if args.config.is_empty() {
                zenoh::Config::default()
            } else {
                zenoh::Config::from_file(args.config.clone())?
            };
            let session = zenoh::open(config).await?;

            let zsub = session.declare_subscriber(ZENOH_KEYEXPR).await?;

            let queue = Arc::new(Queue2::new(10));
            let c_queue = queue.clone();
            tokio::task::spawn(async move {
                while let Ok(sample) = zsub.recv_async().await {
                    let msg = sample.payload().to_bytes();
                    c_queue.push(msg.to_vec()).await;
                }
            });

            Ok(Self {
                _sess: session,
                queue,
            })
        }

        async fn recv(&mut self) -> Result<Timestamp> {
            let msg = self.queue.pop().await?;
            // let sample = self.zsub.recv_async().await?;
            // let msg = sample.payload().to_bytes();
            Ok(Timestamp::from_le_bytes(
                msg[0..size_of::<Timestamp>()].try_into()?,
            ))
        }
    }

    #[allow(unused)]
    struct Queue1<T> {
        queue: ArrayQueue<T>,
        push_semaphore: Semaphore,
        pop_semaphore: Semaphore,
    }

    #[allow(unused)]
    impl<T> Queue1<T> {
        pub fn new(max_size: usize) -> Self {
            Self {
                queue: ArrayQueue::new(max_size),
                push_semaphore: Semaphore::new(max_size),
                pop_semaphore: Semaphore::new(0),
            }
        }

        pub async fn pop(&self) -> Result<T> {
            let permit = self.pop_semaphore.acquire().await?;
            let item = self.queue.pop().unwrap();
            permit.forget();
            self.push_semaphore.add_permits(1);
            Ok(item)
        }

        pub async fn push(&self, item: T) {
            let permit = self.push_semaphore.acquire().await.unwrap();
            self.queue.force_push(item);
            permit.forget();
            self.pop_semaphore.add_permits(1);
        }
    }

    struct Queue2<T> {
        queue: tokio::sync::Mutex<VecDeque<T>>,
        notify: tokio::sync::Notify,
        capacity: usize,
    }

    impl<T> Queue2<T> {
        pub fn new(capacity: usize) -> Self {
            Self {
                queue: tokio::sync::Mutex::new(VecDeque::with_capacity(capacity)),
                notify: tokio::sync::Notify::new(),
                capacity,
            }
        }

        pub async fn push(&self, item: T) {
            let mut queue = self.queue.lock().await;
            if queue.len() >= self.capacity {
                queue.pop_front();
            }
            queue.push_back(item);
            self.notify.notify_waiters();
        }

        pub async fn pop(&self) -> Result<T> {
            loop {
                let mut queue = self.queue.lock().await;
                if let Some(item) = queue.pop_front() {
                    return Ok(item);
                }
                drop(queue);
                self.notify.notified().await;
            }
        }
    }

    // type Queue<T> = Queue1<T>;
    type Queue<T> = Queue2<T>;

    pub struct TokioReceiver {
        // socket: TcpStream,
        // buf: Vec<u8>,
        queue: Arc<Queue<Vec<u8>>>,
    }

    impl Receivable for TokioReceiver {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Tokio
        }

        async fn init(args: Rc<Args>) -> Result<Self> {
            let listener = tokio::net::TcpListener::bind(TOKIO_SERVER_ADDR).await?;
            let (mut socket, addr) = listener.accept().await?;
            // socket.set_nodelay(true)?;
            // let buf = vec![0u8; args.payload];
            let queue = Arc::new(Queue::new(10));
            let len = args.payload;
            println!("Accepted connection from {}", addr);
            let c_queue = queue.clone();
            tokio::task::spawn(async move {
                let mut buf = vec![0u8; 4096];
                while let Ok(n) = socket.read(&mut buf).await {
                    if n == 0 {
                        continue;
                    }
                    // assert!(n % len == 0);
                    for idx in 0..(n / len) {
                        let i = idx * len;
                        let j = i + len;
                        c_queue.push(buf[i..j].to_owned()).await;
                    }
                }
            });
            Ok(Self { queue })
        }

        async fn recv(&mut self) -> Result<Timestamp> {
            let msg = self.queue.pop().await?;
            let x = msg[0..size_of::<Timestamp>()].try_into()?;
            Ok(Timestamp::from_le_bytes(x))
        }
    }

    pub struct TokioUdpReceiver {
        queue: Arc<Queue<Vec<u8>>>,
    }

    impl Receivable for TokioUdpReceiver {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Tokio
        }

        async fn init(args: Rc<Args>) -> Result<Self> {
            print!("Connecting to a UDP socket...");
            let _ = std::io::stdout().flush();
            let socket = UdpSocket::bind(TOKIO_SERVER_ADDR).await?;
            println!("done.");
            let queue = Arc::new(Queue::new(10));
            let len = args.payload;
            let c_queue = queue.clone();
            tokio::task::spawn(async move {
                let mut buf = vec![0u8; len];
                loop {
                    socket.recv(&mut buf).await.unwrap();
                    c_queue.push(buf.clone()).await;
                }
            });
            Ok(Self { queue })
        }

        async fn recv(&mut self) -> Result<Timestamp> {
            let msg = self.queue.pop().await?;
            let x = msg[0..size_of::<Timestamp>()].try_into()?;
            Ok(Timestamp::from_le_bytes(x))
        }
    }

    pub struct GlommioReceiver {
        receiver: LocalReceiver<Vec<u8>>,
        _task: glommio::task::JoinHandle<()>,
    }

    impl Receivable for GlommioReceiver {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Glommio
        }

        async fn init(args: Rc<Args>) -> Result<Self> {
            tracing::info!("Glommio recevier initiating...");
            let listener = glommio::net::TcpListener::bind(GLOMMIO_SERVER_ADDR).unwrap();
            let mut stream = listener.accept().await.unwrap();

            let (sender, receiver) = local_channel::new_bounded::<Vec<u8>>(10);

            use futures::AsyncReadExt;
            let payload_size = args.payload;
            let task = glommio::spawn_local(async move {
                let mut buf = vec![0u8; 4096];
                loop {
                    match stream.read(&mut buf).await {
                        Ok(0) => continue,
                        Ok(n) => {
                            assert!(n % payload_size == 0);
                            for idx in 0..(n / payload_size) {
                                let i = idx * payload_size;
                                let j = i + payload_size;
                                if sender.send(buf[i..j].to_vec()).await.is_err() {
                                    break;
                                }
                            }
                        }
                        Err(_) => break,
                    }
                }
            })
            .detach();

            tracing::info!("Done.");
            Ok(Self {
                receiver,
                _task: task,
            })
        }

        async fn recv(&mut self) -> Result<Timestamp> {
            let msg = self.receiver.recv().await.unwrap();
            let x = msg[0..size_of::<Timestamp>()].try_into()?;
            Ok(Timestamp::from_le_bytes(x))
        }
    }

    pub struct MonoioReceiver {
        receiver: flume::Receiver<Vec<u8>>,
        _task: monoio::task::JoinHandle<()>,
    }

    impl Receivable for MonoioReceiver {
        fn async_runtime() -> AsyncRuntime {
            AsyncRuntime::Monoio
        }

        async fn init(args: Rc<Args>) -> Result<Self> {
            tracing::info!("Monoio recevier initiating...");
            let listener = monoio::net::TcpListener::bind(GLOMMIO_SERVER_ADDR).unwrap();
            let (mut stream, _addr) = listener.accept().await.unwrap();

            let (sender, receiver) = flume::bounded(10);

            use monoio::io::AsyncReadRent;
            let payload_size = args.payload;
            let mut buf = vec![0u8; 4096];
            let task = monoio::spawn(async move {
                loop {
                    buf = match stream.read(buf).await {
                        (Ok(0), buf) => buf,
                        (Ok(n), buf) => {
                            assert!(n % payload_size == 0);
                            for idx in 0..(n / payload_size) {
                                let i = idx * payload_size;
                                let j = i + payload_size;
                                if sender.send_async(buf[i..j].to_vec()).await.is_err() {
                                    break;
                                }
                            }
                            buf
                        }
                        (Err(_), _) => break,
                    };
                }
            });

            tracing::info!("Done.");
            Ok(Self {
                receiver,
                _task: task,
            })
        }

        async fn recv(&mut self) -> Result<Timestamp> {
            let msg = self.receiver.recv_async().await.unwrap();
            let x = msg[0..size_of::<Timestamp>()].try_into()?;
            Ok(Timestamp::from_le_bytes(x))
        }
    }

    pub fn rx_task<R: Receivable>(args: Rc<Args>) -> Result<()> {
        let rt = R::async_runtime();
        tracing::info!("Async runtime: {rt:?}");

        let now = std::time::Instant::now();
        let warmup = std::time::Duration::from_secs(args.warmup as u64);

        let warmup_samples = (args.sample as f64 * 0.2) as usize;

        rt.block_on(async move {
            let mut receiver = R::init(args.clone()).await?;
            tracing::info!("Begining the warm up...");
            std::io::stdout().flush().unwrap();

            let mut counter = 0;
            while now.elapsed() < warmup || counter <= warmup_samples {
                receiver.recv().await?;
                counter += 1;
            }
            tracing::info!("Done.");

            receiver.clean();

            let mut lats = Vec::with_capacity(args.sample);
            // let mut now = std::time::Instant::now();
            while let Ok(sent_time) = receiver.recv().await {
                let now_ns = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .expect("Time went backwards")
                    .as_nanos() as u64;
                // let now_ns = get_timestamp();
                let lat = now_ns - sent_time;
                if lat == 0 {
                    tracing::error!("[rx] lat = 0");
                    continue;
                }
                // println!("[Receiver]: {} - {} = {}, queue size: {}", now_ns, sent_time, lat, zsub.len());
                lats.push(lat as _);
                if lats.len() >= args.sample {
                    break;
                }

                // if now.elapsed() >= std::time::Duration::from_secs(1) {
                //     println!("lats len = {}", lats.len());
                //     now = std::time::Instant::now();
                // }
            }
            print_statistics(lats.clone());

            if !args.log.is_empty() {
                let logger = DataLogger {
                    frequency: args.frequency,
                    payload: args.payload,
                    path: PathBuf::from(args.log.clone()),
                };

                logger.write(lats)?;
            }

            Ok(())
        })
    }
}

#[derive(Debug, Copy, Clone)]
enum AsyncRuntime {
    Tokio,
    Glommio,
    Monoio,
}

impl AsyncRuntime {
    pub fn block_on(&self, fut: impl Future<Output = Result<()>>) -> Result<()> {
        match self {
            AsyncRuntime::Tokio => tokio::runtime::Builder::new_multi_thread()
                .worker_threads(2)
                .enable_time()
                .enable_io()
                .build()?
                .block_on(fut),
            // AsyncRuntime::Tokio => tokio::runtime::Builder::new_current_thread()
            //     .worker_threads(2)
            //     .enable_time()
            //     .enable_io()
            //     .build()?
            //     .block_on(fut),
            AsyncRuntime::Glommio => glommio::LocalExecutor::default().run(fut),
            AsyncRuntime::Monoio => monoio::RuntimeBuilder::<monoio::IoUringDriver>::new()
                .enable_all()
                .build()?
                .block_on(fut),
        }
    }
}

fn main() -> Result<()> {
    let args = Rc::new(Args::parse());

    tracing_subscriber::fmt::init();

    println!(
        "Freq: {} Hz, Payload: {} bytes, Samples: {}",
        &args.frequency, &args.payload, &args.sample
    );

    if args.recv {
        use crate::rx::*;
        match args.mode {
            Mode::Zenoh => rx_task::<ZenohReceiver>(args)?,
            Mode::Tokio => rx_task::<TokioReceiver>(args)?,
            Mode::Udp => rx_task::<TokioUdpReceiver>(args)?,
            Mode::Foo => rx_task::<FooReceiver>(args)?,
            Mode::Glommio => rx_task::<GlommioReceiver>(args)?,
            Mode::Monoio => rx_task::<MonoioReceiver>(args)?,
            Mode::StdTcp => crate::sync::rx_task::<crate::sync::StdTcpReceiver>(args)?,
            Mode::StdUdp => crate::sync::rx_task::<crate::sync::StdUdpReceiver>(args)?,
        }
    } else {
        use crate::tx::*;
        match args.mode {
            Mode::Zenoh => tx_task::<ZenohSender>(args)?,
            Mode::Tokio => tx_task::<TokioSender>(args)?,
            Mode::Udp => tx_task::<TokioUdpSender>(args)?,
            Mode::Foo => tx_task::<ZenohSender>(args)?,
            Mode::Glommio => tx_task::<GlommioSender>(args)?,
            Mode::Monoio => tx_task::<MonoioSender>(args)?,
            Mode::StdTcp => crate::sync::tx_task::<crate::sync::StdTcpSender>(args)?,
            Mode::StdUdp => crate::sync::tx_task::<crate::sync::StdUdpSender>(args)?,
        }
    }
    Ok(())
}
