//! Bounded multi-producer/single-consumer queue with a lock-free consumer.
//!
//! Producer calls are mutex-serialized; only the unique `Consumer` can pop.
//! A full queue rejects the newest item rather than writing the consumer index.
//! Capacity is N-1 and N must be a power of two. No enqueue/dequeue allocations.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Mutex;

pub struct Queue<T: Copy, const N: usize> {
    buf: UnsafeCell<[T; N]>,
    head: AtomicUsize,
    tail: AtomicUsize,
    producer: Mutex<()>,
    claimed: AtomicBool,
    dropped: AtomicUsize,
}

// SAFETY: producer access is serialized. A unique Consumer owns tail; producers
// own head and never overwrite unread slots. Release/acquire publishes each
// slot and prevents reuse until the consumer has finished copying its value.
unsafe impl<T: Copy + Send, const N: usize> Sync for Queue<T, N> {}

impl<T: Copy, const N: usize> Queue<T, N> {
    pub const fn new(empty: T) -> Self {
        assert!(N >= 2 && N.is_power_of_two());
        Self {
            buf: UnsafeCell::new([empty; N]),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            producer: Mutex::new(()),
            claimed: AtomicBool::new(false),
            dropped: AtomicUsize::new(0),
        }
    }

    pub fn push(&self, item: T) -> bool {
        let _guard = self.producer.lock().unwrap_or_else(|e| e.into_inner());
        let head = self.head.load(Ordering::Relaxed);
        let next = (head + 1) & (N - 1);
        if next == self.tail.load(Ordering::Acquire) {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        // SAFETY: this producer owns the unused head slot until head is published.
        unsafe {
            self.buf.get().cast::<T>().add(head).write(item);
        }
        self.head.store(next, Ordering::Release);
        true
    }

    /// Claim the only receiver for this queue. The claim lasts for its lifetime.
    pub fn take_consumer(&self) -> Option<Consumer<'_, T, N>> {
        self.claimed
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
            .ok()
            .map(|_| Consumer { queue: self })
    }

    pub fn dropped(&self) -> usize {
        self.dropped.load(Ordering::Relaxed)
    }
}

pub struct Consumer<'a, T: Copy, const N: usize> {
    queue: &'a Queue<T, N>,
}

impl<T: Copy, const N: usize> Consumer<'_, T, N> {
    pub fn pop(&mut self) -> Option<T> {
        let queue = self.queue;
        let tail = queue.tail.load(Ordering::Relaxed);
        if tail == queue.head.load(Ordering::Acquire) {
            return None;
        }
        // SAFETY: the acquire observes initialization; only this receiver can
        // read/release tail, and producers cannot reuse the slot before release.
        let item = unsafe { queue.buf.get().cast::<T>().add(tail).read() };
        queue.tail.store((tail + 1) & (N - 1), Ordering::Release);
        Some(item)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_wraparound_and_overflow_preserve_unread_items() {
        let queue = Queue::<u32, 4>::new(0);
        let mut receiver = queue.take_consumer().unwrap();
        assert!(queue.take_consumer().is_none());
        assert_eq!(receiver.pop(), None);
        for i in 1..=3 {
            assert!(queue.push(i));
        }
        assert!(!queue.push(4));
        assert_eq!(queue.dropped(), 1);
        assert_eq!(receiver.pop(), Some(1));
        assert!(queue.push(5));
        for value in [2, 3, 5] {
            assert_eq!(receiver.pop(), Some(value));
        }
        assert_eq!(receiver.pop(), None);
    }

    #[test]
    fn concurrent_producers_preserve_each_producers_order() {
        let queue = Queue::<(usize, usize), 64>::new((0, 0));
        let mut receiver = queue.take_consumer().unwrap();
        std::thread::scope(|scope| {
            for producer in 0..3 {
                let queue = &queue;
                scope.spawn(move || {
                    for sequence in 0..1000 {
                        while !queue.push((producer, sequence)) {
                            std::thread::yield_now();
                        }
                    }
                });
            }
            let mut expected = [0; 3];
            for _ in 0..3000 {
                let (producer, sequence) = loop {
                    if let Some(item) = receiver.pop() {
                        break item;
                    }
                    std::thread::yield_now();
                };
                assert_eq!(sequence, expected[producer]);
                expected[producer] += 1;
            }
            assert_eq!(expected, [1000; 3]);
        });
    }
}
