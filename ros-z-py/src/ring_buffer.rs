/// Ring buffer with drop-oldest semantics for ROS 2 KeepLast QoS
pub struct RingBuffer<T> {
    buffer: Vec<Option<T>>,
    head: usize,
    tail: usize,
    size: usize,
    capacity: usize,
}

impl<T> RingBuffer<T> {
    pub fn new(capacity: usize) -> Self {
        Self {
            buffer: (0..capacity).map(|_| None).collect(),
            head: 0,
            tail: 0,
            size: 0,
            capacity,
        }
    }

    pub fn push(&mut self, item: T) {
        if self.size == self.capacity {
            // Drop oldest
            self.tail = (self.tail + 1) % self.capacity;
            self.size -= 1;
        }
        self.buffer[self.head] = Some(item);
        self.head = (self.head + 1) % self.capacity;
        self.size += 1;
    }

    pub fn pop(&mut self) -> Option<T> {
        if self.size == 0 {
            None
        } else {
            let item = self.buffer[self.tail].take();
            self.tail = (self.tail + 1) % self.capacity;
            self.size -= 1;
            item
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_drop_oldest() {
        let mut buf = RingBuffer::new(3);
        buf.push(1);
        buf.push(2);
        buf.push(3);
        buf.push(4); // Should drop 1

        assert_eq!(buf.pop(), Some(2));
        assert_eq!(buf.pop(), Some(3));
        assert_eq!(buf.pop(), Some(4));
        assert_eq!(buf.pop(), None);
    }
}
