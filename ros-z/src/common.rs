use std::sync::Arc;

/// Core abstraction for handling incoming data in subscribers and servers
pub(crate) enum DataHandler<T> {
    /// Queue-based: store for later retrieval
    Queue(flume::Sender<T>),

    /// Direct callback: process immediately
    Callback(Arc<dyn Fn(T) + Send + Sync>),

    /// Queue + notify: store and trigger notification
    #[allow(dead_code)]
    QueueWithNotifier {
        sender: flume::Sender<T>,
        notifier: Arc<dyn Fn() + Send + Sync>,
    },
}

impl<T> DataHandler<T> {
    pub(crate) fn handle(&self, data: T) {
        match self {
            DataHandler::Queue(tx) => {
                let _ = tx.send(data);
            }
            DataHandler::Callback(cb) => cb(data),
            DataHandler::QueueWithNotifier { sender, notifier } => {
                let _ = sender.send(data);
                notifier();
            }
        }
    }
}
