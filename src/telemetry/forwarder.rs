use std::fmt::Write;
use std::sync::mpsc::SyncSender;
use std::time::Instant;

use heapless::{self, String as HString, Vec as HVec};
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::{
    layer::{Context, Layer},
    registry::LookupSpan,
};

/// How many fields we inline per event (must bound it at compile time).
const MAX_FIELDS: usize = 8;
/// Max bytes for any string value, one for newline
const MAX_STR_LEN: usize = 64;

/// One captured field.
#[derive(Debug)]
pub struct FieldRecord {
    pub name: &'static str,
    pub value: FieldValue,
}

/// The value of a field—never allocates.
#[derive(Debug)]
pub enum FieldValue {
    U64(u64),
    I64(i64),
    Bool(bool),
    Str(String),
    Debug(HString<MAX_STR_LEN>),
}

/// One “perfectly forwarded” event.
#[derive(Debug)]
pub struct EventRecord {
    pub ts: Instant,
    pub target: &'static str,
    pub level: Level,
    pub fields: HVec<FieldRecord, MAX_FIELDS>,
}

/// Visitor that slurps up _every_ field into our heapless Vec.
struct FieldCollector<'a>(&'a mut HVec<FieldRecord, MAX_FIELDS>);

impl<'a> Visit for FieldCollector<'a> {
    fn record_u64(&mut self, field: &Field, value: u64) {
        let _ = self.0.push(FieldRecord {
            name: field.name(),
            value: FieldValue::U64(value),
        });
    }
    fn record_i64(&mut self, field: &Field, value: i64) {
        let _ = self.0.push(FieldRecord {
            name: field.name(),
            value: FieldValue::I64(value),
        });
    }
    fn record_bool(&mut self, field: &Field, value: bool) {
        let _ = self.0.push(FieldRecord {
            name: field.name(),
            value: FieldValue::Bool(value),
        });
    }
    fn record_str(&mut self, field: &Field, value: &str) {
        let _ = self.0.push(FieldRecord {
            name: field.name(),
            value: FieldValue::Str(String::from(value)),
        });
    }
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        let mut s = HString::new();
        // format into inline buffer
        let _ = write!(&mut s, "{value:?}");
        let _ = self.0.push(FieldRecord {
            name: field.name(),
            value: FieldValue::Debug(s),
        });
    }
}

pub struct HeaplessForwardLayer {
    pub tx: SyncSender<EventRecord>,
}

impl<S> Layer<S> for HeaplessForwardLayer
where
    S: Subscriber + for<'lookup> LookupSpan<'lookup>,
{
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        // vector to store fields
        let mut fields = HVec::<FieldRecord, MAX_FIELDS>::new();
        // record all fields into the event
        {
            let mut collector = FieldCollector(&mut fields);
            event.record(&mut collector);
        }

        // create a record of the event
        let rec = EventRecord {
            ts: Instant::now(),
            target: event.metadata().target(),
            level: *event.metadata().level(),
            fields,
        };

        // send the record
        let _ = self.tx.send(rec);
    }
}
