# firmware

The original firmware for kbot has a few issues. Notably:
1. A lot of dynamic memory allocations in the hot path (e.g. `.collect(), Box::new()`, etc.)
2. Threads are spawned in a less controlled manner.

This rewrite aims to minimize dynamic memory allocations and stick to a single-threaded runtime for now. We can expand to a multithreaded one with more control

This is the first commit that is tested on the bot. Work is still pending.

# Todo's


## Short Term
- [ ] Let's make a development branch for nightly
- [ ] keyboard support
- [ ] Trait to auto impl streaming state machine
- [ ] handle faults from actuators in a more graceful manner
- [ ] merge files into small crates and change this repo into a **single** workspace

## Medium Term
- [ ] setup instrumentation and tracing

## Long Term
- [ ] Use zenoh for shared memory queues between processes
