use std::io::{self, Read};
use std::slice;

/// A “page” whose backing storage lives elsewhere.
/// supply a pointer + capacity, and this struct
/// it tracks `len` (how much is filled) and gives
/// safe slices into the valid portion or the free tail.
#[derive(Debug, Clone)]
pub struct Page {
    ptr: *mut u8,
    cap: usize,
    len: usize,
}

impl Page {
    /// # Safety
    /// - `ptr` must point to `cap` bytes of writable memory,
    ///   valid for the lifetime of this Page.
    /// - No other aliasing mutable borrows may exist.
    pub unsafe fn new(ptr: *mut u8, cap: usize) -> Self {
        Page { ptr, cap, len: 0 }
    }

    /// How many bytes have been filled so far.
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Total capacity (in bytes).
    pub fn capacity(&self) -> usize {
        self.cap
    }

    /// How many bytes remain free.
    pub fn remaining(&self) -> usize {
        self.cap - self.len
    }

    /// A `&[u8]` of the filled portion.
    pub fn as_slice(&self) -> &[u8] {
        // SAFETY: we know that `ptr` is valid for `cap` bytes,
        // and `len <= cap`.
        unsafe { slice::from_raw_parts(self.ptr, self.len) }
    }

    /// A `&mut [u8]` of the unfilled tail.
    pub fn tail_mut(&mut self) -> &mut [u8] {
        // SAFETY: ptr.add(len) is within the same allocation
        unsafe { slice::from_raw_parts_mut(self.ptr.add(self.len), self.remaining()) }
    }

    /// Advance the “filled” counter by `n` bytes.
    ///  
    /// # Panics
    /// If you advance past capacity.
    pub fn advance(&mut self, n: usize) {
        assert!(self.len + n <= self.cap, "overflow page capacity");
        self.len += n;
    }

    /// Read up to `remaining()` bytes from `reader` into the tail,
    /// bumping `len` by how many bytes were read.
    pub fn fill_from<R: Read>(&mut self, rdr: &mut R) -> io::Result<usize> {
        if self.remaining() == 0 {
            return Ok(0);
        }
        let n = rdr.read(self.tail_mut())?;
        self.advance(n);
        Ok(n)
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.ptr
    }

    /// Reset the page to “empty” (but does *not* zero it).
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

#[derive(Debug, Clone)]
pub struct RingEntry {
    pub page: Page,
    pub idx: usize,
}

impl RingEntry {
    pub fn new(page: Page, idx: usize) -> Self {
        RingEntry { page, idx }
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.page.as_ptr()
    }

    pub fn len(&self) -> usize {
        self.page.len()
    }

    pub fn is_empty(&self) -> bool {
        self.page.is_empty()
    }

    pub fn idx(&self) -> usize {
        self.idx
    }

    pub fn fill_from<R: std::io::Read>(&mut self, rdr: &mut R) -> io::Result<usize> {
        self.page.fill_from(rdr)
    }
}
