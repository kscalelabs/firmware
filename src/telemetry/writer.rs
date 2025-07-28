use std::io::{self, Write};
use tracing_subscriber::fmt::MakeWriter;

/// A `MakeWriter` for `tracing-subscriber` that is compatible with terminal raw mode.
///
/// This writer wraps `std::io::stdout` and replaces every `\n` with `\r\n` to ensure
/// that the cursor returns to the beginning of the line after a line feed.
#[derive(Clone)]
pub struct RawModeWriter;

impl<'a> MakeWriter<'a> for RawModeWriter {
    type Writer = impl io::Write + 'a;

    fn make_writer(&'a self) -> Self::Writer {
        // This inner struct does the actual writing.
        struct Writer {
            stdout: io::Stdout,
        }

        impl io::Write for Writer {
            fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
                let mut last_pos = 0;
                // Iterate over the buffer and find all newline characters.
                for (i, &byte) in buf.iter().enumerate() {
                    if byte == b'\n' {
                        // Write the segment before the newline.
                        self.stdout.write_all(&buf[last_pos..i])?;
                        // Write the corrected newline sequence.
                        self.stdout.write_all(b"END")?;
                        self.stdout.write_all(b"\r\n")?;
                        last_pos = i + 1;
                    }
                }
                // Write any remaining part of the buffer after the last newline.
                if last_pos < buf.len() {
                    self.stdout.write_all(&buf[last_pos..])?;
                }

                // We must report that we wrote the original number of bytes,
                // otherwise `tracing` might think the write was incomplete.
                Ok(buf.len())
            }

            fn flush(&mut self) -> io::Result<()> {
                self.stdout.flush()
            }
        }

        Writer { stdout: io::stdout() }
    }
}
