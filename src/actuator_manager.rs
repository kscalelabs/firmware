// #![allow(unused)]

use crate::actuator::ActuatorBus;
use crate::state_machine;
use tracing::{error, info};

use crate::robstride_utils::RobstrideActuatorParam;
use futures::Stream;
use futures::StreamExt;
use futures::TryStream;
use futures::TryStreamExt;
use pin_project::pin_project;
use std::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

use enum_map::{Enum, EnumMap};

use crate::robot_description::{ActuatorStateStore, BusTag};

use std::task::ready;

use crate::actuator;

state_machine!(Reset, Scanning, Ready, Operate);

#[derive(Debug)]
#[pin_project]
struct ActuatorBusWrapper {
    #[pin]
    bus: actuator::ActuatorBus,
    iface_idx: usize,
}

impl ActuatorBusWrapper {
    pub fn reset_iface(&mut self, iface_name: &str, iface_idx: usize) {
        self.bus.reset_iface(iface_name);
        self.iface_idx = iface_idx;
    }
}

#[derive(Debug)]
#[pin_project]
pub struct Store {
    // leftarm_store: actuator::Store,
    #[pin]
    bus_wrappers: Vec<(BusTag, ActuatorBusWrapper)>,
    // bus_wrappers: [ActuatorBusWrapper; 4], // left arm, right arm, left leg, right leg
    // leftarm: Option<ActuatorBus>,
    // rightarm: Option<ActuatorBus>,
    iface_names: Vec<String>,
    av_iface_idxs: std::collections::VecDeque<usize>,
}

impl Default for Store {
    fn default() -> Self {
        Self::new()
    }
}

impl Store {
    pub fn new() -> Self {
        // Build CAN interface list from environment or system; fallback to a sensible default
        fn discover_can_interfaces() -> Vec<String> {
            // environment variable override
            let from_env = std::env::var("KSCALE_CAN_INTERFACES")
                .or_else(|_| std::env::var("CAN_INTERFACES"))
                .ok()
                .map(|val| {
                    val.split(',')
                        .filter_map(|s| {
                            let name = s.trim();
                            if name.is_empty() { None } else { Some(name.to_string()) }
                        })
                        .collect::<Vec<_>>()
                });

            if let Some(list) = from_env {
                if !list.is_empty() {
                    return list;
                }
            }

            // discover from /sys/class/net for can/vcan interfaces
            let mut discovered: Vec<String> = std::fs::read_dir("/sys/class/net")
                .ok()
                .into_iter()
                .flatten()
                .filter_map(|res| res.ok())
                .filter_map(|entry| entry.file_name().into_string().ok())
                .filter(|name: &String| name.starts_with("can") || name.starts_with("vcan"))
                .collect();

            // sort by prefix and numeric suffix if present (can0 < can1 < can10)
            discovered.sort_by(|a, b| {
                fn split_name(s: &str) -> (&str, Option<u32>) {
                    let (prefix, digits) = s.split_at(s.find(|c: char| c.is_ascii_digit()).unwrap_or(s.len()));
                    let num = digits.parse::<u32>().ok();
                    (prefix, num)
                }
                let (pa, na) = split_name(a);
                let (pb, nb) = split_name(b);
                match pa.cmp(pb) {
                    std::cmp::Ordering::Equal => na.cmp(&nb),
                    other => other,
                }
            });

            if !discovered.is_empty() {
                return discovered;
            }

            // default to can0..can6 to support both 5 and 7 bus hardware
            (0..=6).map(|i| format!("can{}", i)).collect::<Vec<_>>()
        }

        // 1) Gather candidates (env list or discovery)
        let discovered = discover_can_interfaces();
        let env_list = std::env::var("KSCALE_CAN_INTERFACES")
            .or_else(|_| std::env::var("CAN_INTERFACES"))
            .ok()
            .map(|val| {
                val.split(',')
                    .filter_map(|s| {
                        let name = s.trim();
                        if name.is_empty() { None } else { Some(name.to_string()) }
                    })
                    .collect::<Vec<_>>()
            }).expect("CAN_INTERFACES is not set correctly, and KSCALE_CAN_LIMB_MAP is not provided");

        let is_upper_body_only = env_list.len() == 2;

        // 2) Optional explicit limb->iface map via KSCALE_CAN_LIMB_MAP
        //    Format: "LeftArm=can6,RightArm=can1,LeftLeg=can3,RightLeg=can2"
        let limb_map_raw = std::env::var("KSCALE_CAN_LIMB_MAP").ok();
        let mut mapped: [Option<String>; 4] = [None, None, None, None];
        if let Some(cfg) = limb_map_raw.as_deref() {
            for entry in cfg.split(',') {
                let kv: Vec<&str> = entry.splitn(2, '=').collect();
                if kv.len() != 2 { continue; }
                let key = kv[0].trim();
                let val = kv[1].trim();
                if val.is_empty() { continue; }
                let idx_opt = match key.to_ascii_lowercase().as_str() {
                    "leftarm" => Some(crate::robot_description::BusTag::LeftArm as usize),
                    "rightarm" => Some(crate::robot_description::BusTag::RightArm as usize),
                    "leftleg" => Some(crate::robot_description::BusTag::LeftLeg as usize),
                    "rightleg" => Some(crate::robot_description::BusTag::RightLeg as usize),
                    _ => None,
                };
                if let Some(idx) = idx_opt { mapped[idx] = Some(val.to_string()); }
            }
        }

        let have_full_limb_map = mapped.iter().all(|v| v.is_some());

        // 3) Build final iface_names: first four entries are limbs in BusTag order.
        let mut iface_names: Vec<String> = if have_full_limb_map {
            info!("Using KSCALE_CAN_LIMB_MAP for limb assignment: {}", limb_map_raw.as_deref().unwrap_or(""));
            mapped.into_iter().map(|o| o.unwrap()).collect()
        } else {
            env_list.clone()
        };

        // 4) Append spares: prefer remaining from env list, else from discovery
        let mut seen = std::collections::HashSet::new();
        for n in &iface_names { seen.insert(n.clone()); }
        for n in env_list {
            if !seen.contains(&n) { iface_names.push(n.clone()); seen.insert(n); }
        }
        for n in discovered {
            if !seen.contains(&n) { iface_names.push(n.clone()); seen.insert(n); }
        }

        // 5) Log final order and per-limb mapping
        info!("Final CAN interface order: {:?}", iface_names);
        info!(
            "Limb->iface: LeftArm={}, RightArm={}, LeftLeg={}, RightLeg={}",
            iface_names.get(crate::robot_description::BusTag::LeftArm as usize).unwrap_or(&"<missing>".to_string()),
            iface_names.get(crate::robot_description::BusTag::RightArm as usize).unwrap_or(&"<missing>".to_string()),
            iface_names.get(crate::robot_description::BusTag::LeftLeg as usize).unwrap_or(&"<missing>".to_string()),
            iface_names.get(crate::robot_description::BusTag::RightLeg as usize).unwrap_or(&"<missing>".to_string()),
        );

        let actuator_ids = [
            BusTag::LeftArm.id_vec(),
            BusTag::RightArm.id_vec(),
            BusTag::LeftLeg.id_vec(),
            BusTag::RightLeg.id_vec(),
        ];

        let mut bus_wrappers: Vec<(BusTag, ActuatorBusWrapper)> = Vec::with_capacity(4);

        // loop through iface names and create bus wrappers (could be 2 or 4 long)
        for (idx, iface_name) in iface_names.iter().enumerate() {
            let ids = &actuator_ids[idx];
            bus_wrappers.push((
                BusTag::from_usize(idx),
                ActuatorBusWrapper {
                    bus: actuator::ActuatorBus::new(iface_name, ids.clone()),
                    iface_idx: idx,
                }
            ));
        }

        // Any interfaces beyond the first 4 are considered spares that can be rotated in on fault
        let mut av_iface_idxs = std::collections::VecDeque::new();
        if iface_names.len() > 4 {
            for idx in 4..iface_names.len() {
                av_iface_idxs.push_back(idx);
            }
        }

        Self { bus_wrappers, iface_names, av_iface_idxs }
    }
}

#[derive(Debug)]
pub struct Reset {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Scanning {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Ready {
    shared_state: Pin<Box<Store>>,
}

#[derive(Debug)]
pub struct Operate {
    shared_state: Pin<Box<Store>>,
}

use std::io;
use std::time::Duration;
use tokio::time::timeout;

impl Ready {
    pub async fn enable(&mut self) -> std::io::Result<()> {
        let mut ss = self.shared_state.as_mut().project();

        let wrappers = unsafe { Pin::get_unchecked_mut(ss.bus_wrappers) };
        for (_, wrapper) in wrappers.iter_mut() {
            // all buses should be operational
            if let Some(actuator::StateStore::Ready(rdy_bus)) = wrapper.bus.get_state() {
                // enable the bus
                rdy_bus.enable().await?;
            } else {
                return Err(io::Error::other("Bus is not in Operate state"));
            }
        }
        Ok(())
    }
}

impl State for Ready {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;
            let mut ss = shared_state.as_mut().project();
            // SAFETY: we know `bus_wrappers` is #[pin], so its elements live
            // in place and can be reborrowed safely.
            let wrappers = unsafe { Pin::get_unchecked_mut(ss.bus_wrappers) };
            for (_, wrapper) in wrappers.iter_mut() {
                wrapper.bus.set_target(actuator::StateTag::Operate);
            }

            // First pass: collect all the pin projections
            let mut bus_pins: Vec<Pin<&mut ActuatorBus>> = Vec::new();
            for (_, wrapper) in wrappers.iter_mut() {
                let wrapper_pin = unsafe { Pin::new_unchecked(wrapper) };
                bus_pins.push(wrapper_pin.project().bus);
            }

            // Second pass: create futures from the long-lived pins
            let futures: Vec<_> = bus_pins.iter_mut().map(|pin| pin.try_next()).collect();
            let results = futures::future::join_all(futures).await;

            info!("Results: {:?}", results);

            let mut proceed = true;
            for (i, result) in results.into_iter().enumerate() {
                match result {
                    Ok(Some(tag)) => {
                        proceed &= tag == actuator::StateTag::Operate;
                        info!(
                            "Bus {} reached {:?} on iface {}",
                            i,
                            tag,
                            ss.iface_names[wrappers[i].1.iface_idx]
                        );
                    }
                    Ok(None) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset { shared_state }),
                            result: Err(io::Error::other("No data received from bus")),
                        };
                    }
                    Err(e) => {
                        return StateTransitionResult {
                            state: StateStore::Ready(Ready { shared_state }),
                            result: Err(e),
                        };
                    }
                }
            }

            if !proceed {
                return StateTransitionResult {
                    state: StateStore::Ready(Ready { shared_state }),
                    result: Ok(()),
                };
            }

            StateTransitionResult {
                state: StateStore::Operate(Operate { shared_state }),
                result: Ok(()),
            }
        }
    }
}

impl State for Scanning {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;
            let mut ss = shared_state.as_mut().project();
            // SAFETY: we know `bus_wrappers` is #[pin], so its elements live
            // in place and can be reborrowed safely.
            let wrappers = unsafe { Pin::get_unchecked_mut(ss.bus_wrappers) };
            for (_, wrapper) in wrappers.iter_mut() {
                wrapper.bus.set_target(actuator::StateTag::Operate);
            }

            // First pass: collect all the pin projections
            let mut bus_pins: Vec<Pin<&mut ActuatorBus>> = Vec::new();
            for (_, wrapper) in wrappers.iter_mut() {
                let wrapper_pin = unsafe { Pin::new_unchecked(wrapper) };
                bus_pins.push(wrapper_pin.project().bus);
            }

            // Second pass: create futures from the long-lived pins
            let futures: Vec<_> = bus_pins.iter_mut().map(|pin| pin.try_next()).collect();
            let results = futures::future::join_all(futures).await;

            info!("Results: {:?}", results);

            let mut proceed = true;
            for (i, result) in results.into_iter().enumerate() {
                match result {
                    Ok(Some(tag)) => {
                        proceed &= tag == actuator::StateTag::Ready;
                        info!(
                            "Bus {} reached {:?} on iface {}",
                            i, tag, ss.iface_names[wrappers[i].1.iface_idx]
                        );
                    }
                    Ok(None) => {
                        return StateTransitionResult {
                            state: StateStore::Reset(Reset { shared_state }),
                            result: Err(io::Error::other("No data received from bus")),
                        };
                    }
                    Err(e) => {
                        error!("Error polling bus {}: {:?}", i, e);
                        // handle error, e.g. reset the bus
                        if let Some(av_idx) = ss.av_iface_idxs.pop_front() {
                            ss.av_iface_idxs.push_back(wrappers[i].1.iface_idx);
                            error!(
                                "Resetting bus {} from {} to {}",
                                i, ss.iface_names[wrappers[i].1.iface_idx], ss.iface_names[av_idx]
                            );
                            wrappers[i].1.reset_iface(ss.iface_names[av_idx].as_str(), av_idx);
                            proceed = false;
                        } else {
                            error!(
                                "No spare CAN interfaces available to reset bus {} (staying on {})",
                                i, ss.iface_names[wrappers[i].1.iface_idx]
                            );
                            proceed = false;
                        }
                    }
                }
            }

            if !proceed {
                return StateTransitionResult {
                    state: StateStore::Scanning(Scanning { shared_state }),
                    result: Ok(()),
                };
            }

            StateTransitionResult {
                state: StateStore::Ready(Ready { shared_state }),
                result: Ok(()),
            }
        }
    }
}

impl State for Reset {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;
            StateTransitionResult {
                state: StateStore::Scanning(Scanning { shared_state }),
                result: Ok(()),
            }
        }
    }
}

impl Operate {
    pub fn get_bus_count(&self) -> usize {
        let ss = self.shared_state.as_ref().project_ref();
        ss.bus_wrappers.len()
    }

    pub async fn request_feedback(&mut self) -> std::io::Result<()> {
        let mut ss = self.shared_state.as_mut().project();

        let wrappers = unsafe { Pin::get_unchecked_mut(ss.bus_wrappers) };
        for (_, wrapper) in wrappers.iter_mut() {
            // all buses should be operational
            if let Some(actuator::StateStore::Operate(op_bus)) = wrapper.bus.get_state() {
                // enable the bus
                op_bus.request_feedback().await?;
            } else {
                return Err(io::Error::other("Bus is not in Operate state"));
            }
        }
        Ok(())
    }

    pub async fn process_feedback(
        &mut self,
        act_states: &mut ActuatorStateStore,
    ) -> std::io::Result<()> {
        let mut ss = self.shared_state.as_mut().project();

        let wrappers = unsafe { Pin::get_unchecked_mut(ss.bus_wrappers) };
        for (i, (tag, wrapper)) in wrappers.iter_mut().enumerate() {
            // all buses should be operational
            if let Some(actuator::StateStore::Operate(op_bus)) = wrapper.bus.get_state() {
                // process the feedback
                op_bus
                    .process_feedback(act_states.slice_mut(*tag))
                    .await?;
            } else {
                return Err(io::Error::other("Bus is not in Operate state"));
            }
        }
        Ok(())
    }

    pub async fn send_command(&mut self, act_states: &ActuatorStateStore) -> std::io::Result<()> {
        let mut ss = self.shared_state.as_mut().project();

        let wrappers = unsafe { Pin::get_unchecked_mut(ss.bus_wrappers) };
        for (tag, wrapper) in wrappers.iter_mut() {
            // all buses should be operational
            if let Some(actuator::StateStore::Operate(op_bus)) = wrapper.bus.get_state() {
                // enable the bus
                op_bus.command(act_states.slice(*tag)).await?;
            } else {
                return Err(io::Error::other("Bus is not in Operate state"));
            }
        }
        Ok(())
    }

    // NOTE: the param should be actuator agnostic
    // to do this, we can have a superset enum which holds a subset enum
    // enum ActuatorParam {
    //  RobstrideActuatorParam(RobstrideActuatorParam),
    //  FeetechActuatorParam(FeetechActuatorParam),
    //  etc.
    //
    //  The backend then handles the appropriate param
    //
    //  For now we will just use RobstrideActuatorParam
    pub async fn request_param(&mut self, param: RobstrideActuatorParam) -> std::io::Result<()> {
        let mut ss = self.shared_state.as_mut().project();

        let wrappers = unsafe { Pin::get_unchecked_mut(ss.bus_wrappers) };
        for (_, wrapper) in wrappers.iter_mut() {
            // all buses should be operational
            if let Some(actuator::StateStore::Operate(op_bus)) = wrapper.bus.get_state() {
                // enable the bus
                op_bus.request_param(param).await?;
            } else {
                return Err(io::Error::other("Bus is not in Operate state"));
            }
        }
        Ok(())
    }

    // pub async fn drive_buses(&mut self) -> std::io::Result<()> {
    //     let mut ss = self.shared_state.as_mut().project();

    //     let wrappers = unsafe { Pin::get_unchecked_mut(ss.bus_wrappers) };
    //     for wrapper in wrappers.values_mut() {
    //         // all buses should be operational
    //         if let Some(actuator::StateStore::Operate(op_bus)) = wrapper.bus.get_state() {
    //             // enable the bus
    //             op_bus.command(42.0).await?;
    //         } else {
    //             return Err(io::Error::new(io::ErrorKind::Other, "Bus is not in Operate state"));
    //         }
    //     };
    //     Ok(())
    //     /*
    //     let mut ss = self.shared_state.as_mut().project();

    //     // SAFETY: we know `bus_wrappers` is #[pin], so its elements live
    //     // in place and can be reborrowed safely.
    //     let wrappers = unsafe { Pin::get_unchecked_mut(ss.bus_wrappers) };
    //     // unsafe { std::mem::transmute(&mut *ss.bus_wrappers) };

    //     let mut op_buses = std::array::from_fn(|i| {
    //         let wrapper = &wrappers[i];
    //         let actuator::StateStore::Operate(op_bus) = wrapper.bus.get_state().unwrap() else {
    //             panic!("Expected bus to be in Operate state");
    //         };
    //         op_bus
    //     });

    //     let [f0, f1, f2, f3] = [
    //         op_buses[0].command(42.0),
    //         op_buses[1].command(42.0),
    //         op_buses[2].command(42.0),
    //         op_buses[3].command(42.0),
    //     ];

    //     // let (r0, r1, r2, r3) = tokio::join!(f0, f1, f2, f3);
    //     let results = tokio::join!(f0, f1, f2, f3);
    //     let results = [
    //         results.0,
    //         results.1,
    //         results.2,
    //         results.3,
    //     ];

    //     info!("Results: {:?}", results);

    //     for (i, results) in results.into_iter().enumerate() {
    //         match results {
    //             Ok(tag) => {
    //                 log::info!("Bus {} reached {:?} on iface {}", i, tag, ss.iface_names[wrappers[i].iface_idx]);
    //             }
    //             Err(e) => {
    //                 log::error!("Error polling bus {}: {:?}", i, e);
    //                 // handle error, e.g. reset the bus
    //                 let av_idx = ss.av_iface_idxs.pop_front().expect("Expected at least 4 actuator buses");
    //                 ss.av_iface_idxs.push_back(wrappers[i].iface_idx);
    //                 log::error!("Resetting bus {} from {} to {}", i, ss.iface_names[wrappers[i].iface_idx], ss.iface_names[av_idx]);
    //                 wrappers[i].reset_iface(ss.iface_names[av_idx].as_str(), av_idx);
    //             }
    //         }
    //     }
    //     */
    // }
}

impl State for Operate {
    #[allow(clippy::manual_async_fn)]
    fn transition_fut(self) -> impl std::future::Future<Output = StateTransitionResult> {
        async move {
            let mut shared_state = self.shared_state;

            StateTransitionResult {
                state: StateStore::Operate(Operate { shared_state }),
                result: Ok(()),
            }
        }
    }
}

impl std::fmt::Debug for ActuatorManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ActuatorManager")
            .field("state", &self.state)
            .field("target", &self.target)
            .finish()
    }
}

#[pin_project]
pub struct ActuatorManager {
    state: Option<StateStore>,
    target: Option<StateTag>,

    #[pin]
    pending_fut: Option<StateFut>,
}

impl Default for ActuatorManager {
    fn default() -> Self {
        Self::new()
    }
}

impl ActuatorManager {
    pub fn new() -> Self {
        Self {
            state: Some(StateStore::Reset(Reset {
                shared_state: Box::pin(Store::new()),
            })),
            target: None,
            pending_fut: None,
        }
    }

    pub fn set_target_pinned(self: Pin<&mut Self>, target: StateTag) {
        *self.project().target = Some(target);
    }

    pub fn get_state_pinned(self: Pin<&mut Self>) -> Option<&mut StateStore> {
        self.project().state.as_mut()
    }

    pub fn reset_state(&mut self) -> std::io::Result<()> {
        if self.state.is_none() {
            return Err(io::Error::other("No state to reset"));
        }

        self.state = Some(match self.state.take().unwrap() {
            StateStore::Reset(_) => StateStore::Reset(Reset {
                shared_state: Box::pin(Store::new()),
            }),
            StateStore::Scanning(scanning) => StateStore::Reset(Reset {
                shared_state: scanning.shared_state,
            }),
            StateStore::Ready(ready) => StateStore::Reset(Reset {
                shared_state: ready.shared_state,
            }),
            StateStore::Operate(operate) => StateStore::Reset(Reset {
                shared_state: operate.shared_state,
            }),
        });

        self.target = None;
        self.pending_fut = None;
        Ok(())
    }
}

impl Stream for ActuatorManager {
    type Item = std::io::Result<StateTag>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let mut this = self.project();

        if let Some(pending_fut) = this.pending_fut.as_mut().as_pin_mut() {
            // log::debug!("Polling pending future: {:?}", pending_fut);
            // If the pending future is ready, we can transition to the next state
            let StateTransitionResult { state: st, result } = ready!(pending_fut.poll(cx));
            // clear the pending future
            unsafe {
                *this.pending_fut.get_unchecked_mut() = None;
            }

            let tag = st.tag();
            *this.state = Some(st); // Update the state
            if let Err(e) = result {
                error!("State transition failed: {:?}", e);
                return Poll::Ready(Some(Err(e)));
            }
            return Poll::Ready(Some(Ok(tag)));
        }

        // check if we have laready reached the specified target
        if let Some(target) = this.target {
            let current = this.state.as_ref().unwrap().tag();
            if &current == target {
                return Poll::Ready(Some(Ok(current)));
            }
        }

        // start new transition to reach the target state
        unsafe {
            *this.pending_fut.get_unchecked_mut() = Some(
                this.state
                    .take()
                    .expect("state must not be None")
                    .transition_fut(),
            );
        }

        // we return pending for this cycle
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}
