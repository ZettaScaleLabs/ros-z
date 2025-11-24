#![allow(unused)]

use std::collections::{HashMap, VecDeque};
use std::ptr::null;
use std::sync::Arc;

use crate::init::rcl_get_default_allocator;
use crate::pubsub::SubscriptionImpl;
use crate::service::{ClientImpl, ServiceImpl};
use crate::traits::HasImplPtr;
use crate::traits::Waitable;
use crate::utils::Notifier;
use crate::{
    c_void, impl_has_impl_ptr,
    ros::*,
    traits::{BorrowImpl, OwnImpl},
};
use parking_lot::lock_api::MutexGuard;
use std::time::Duration;
use strum::{EnumIter, IntoEnumIterator};
use zenoh::Result;

// NOTE: The iterating order matters!
#[derive(Debug, Hash, PartialEq, Eq, PartialOrd, Ord, Clone, Copy, EnumIter)]
pub enum WaitSetKind {
    Subscription,
    Client,
    Service,
    GuradCondition,
    Timer,
    Event,
}

type WaitSetQueue = Vec<*const c_void>;

#[derive(Debug)]
pub struct WaitSetImpl {
    hmap: HashMap<WaitSetKind, WaitSetQueue>,
    mirror: Option<HashMap<WaitSetKind, Box<[*const c_void]>>>,
    notifier: Option<Arc<Notifier>>,
    allocator: rcl_allocator_t,
}

impl WaitSetImpl {
    fn new() -> Self {
        Self {
            hmap: WaitSetKind::iter()
                .map(|x| (x, WaitSetQueue::new()))
                .collect(),
            mirror: None,
            notifier: None,
            allocator: rcl_get_default_allocator(),
        }
    }

    // fn resize(&mut self, key: &WaitSetKind, len: usize) {
    //     match self.hmap.get_mut(key) {
    //         Some(queue) => {
    //             queue.resize(len, std::ptr::null());
    //         }
    //         None => {
    //             let mut x = WaitSetQueue::new();
    //             x.resize(len, std::ptr::null());
    //             self.hmap.insert(*key, x);
    //         }
    //     }
    // }

    fn insert(&mut self, key: &WaitSetKind, val: *const c_void) {
        match self.hmap.get_mut(key) {
            Some(queue) => {
                queue.push(val);
            }
            None => {
                self.hmap.insert(*key, WaitSetQueue::from([val]));
            }
        }
    }

    fn queue_len(&self, key: &WaitSetKind) -> usize {
        match self.hmap.get(key) {
            Some(queue) => queue.len(),
            None => 0,
        }
    }

    fn reset_all(&mut self) {
        self.hmap.iter_mut().for_each(|(k, v)| {
            *v = WaitSetQueue::default();
        });
    }

    fn is_ready<T>(&self, kind: WaitSetKind) -> bool
    where
        T: HasImplPtr<ImplType: Waitable>,
    {
        match self.hmap.get(&kind) {
            Some(queue) => queue
                .iter()
                .any(|&ptr| match (ptr as *const T).borrow_impl() {
                    Ok(x) => x.is_ready(),
                    Err(err) => {
                        tracing::error!("is_ready failed due to {err}");
                        false
                    }
                }),
            None => false,
        }
    }

    fn is_event_ready(&self) -> bool {
        use crate::event::EVENT_MAP;
        match self.hmap.get(&WaitSetKind::Event) {
            Some(queue) => queue.iter().any(|&ptr| {
                if ptr.is_null() {
                    return false;
                }
                let event_ptr = ptr as usize;
                if let Some(map) = EVENT_MAP.get() {
                    if let Ok(map) = map.lock() {
                        if let Some(event_impl) = map.get(&event_ptr) {
                            // Check if the event has unread status
                            let rmw_handle = &*event_impl.rmw_handle;
                            rmw_handle.is_ready()
                        } else {
                            false
                        }
                    } else {
                        false
                    }
                } else {
                    false
                }
            }),
            None => false,
        }
    }

    fn wait(&self, notifier: &Arc<Notifier>, timeout: Duration) {
        let is_ready = || {
            self.is_ready::<rcl_subscription_t>(WaitSetKind::Subscription)
                || self.is_ready::<rcl_client_t>(WaitSetKind::Client)
                || self.is_ready::<rcl_service_t>(WaitSetKind::Service)
                || self.is_ready::<rcl_guard_condition_t>(WaitSetKind::GuradCondition)
                || self.is_ready::<rcl_timer_t>(WaitSetKind::Timer)
                || self.is_event_ready()
        };

        let (mutex, cv) = {
            let x = notifier.as_ref();
            (&x.mutex, &x.cv)
        };

        let mut lock = mutex.lock();
        cv.wait_while_for(&mut lock, |_| !is_ready(), timeout);
    }

    fn check_ready(&mut self) -> Result<bool> {
        fn check<T>(queue: &mut WaitSetQueue) -> Result<bool>
        where
            T: HasImplPtr<ImplType: Waitable>,
        {
            let mut ready = false;
            for mut ptr in queue {
                if (*ptr as *const T).borrow_impl()?.is_ready() {
                    ready = true;
                } else {
                    *ptr = std::ptr::null();
                }
            }
            Ok(ready)
        }

        let mut ready = false;
        // FIXME: Can we remove the order here?
        for (k, f) in WaitSetKind::iter().zip([
            check::<rcl_subscription_t>,
            check::<rcl_client_t>,
            check::<rcl_service_t>,
            check::<rcl_guard_condition_t>,
            check::<rcl_timer_t>,
        ]) {
            if let Some(queue) = self.hmap.get_mut(&k) {
                if f(queue)? {
                    ready = true;
                }
            }
        }

        // Check events separately
        if self.is_event_ready() {
            ready = true;
        }

        Ok(ready)
    }

    fn mirror(&mut self) {
        self.mirror = Some(
            self.hmap
                .iter()
                .map(|(&k, v)| (k, v.clone().into_boxed_slice()))
                .collect(),
        );
    }

    fn reset_guard_conditions(&mut self) {
        if let Some(queue) = self.hmap.get_mut(&WaitSetKind::GuradCondition) {
            queue.iter().for_each(|&gc| {
                if let Ok(gc_impl) = (gc as *mut rcl_guard_condition_t).borrow_mut_impl() {
                    gc_impl.reset();
                }
            });
        }
    }

    fn min_timeout(&self) -> Duration {
        let mut timeout = Duration::MAX;

        if let Some(queue) = self.hmap.get(&WaitSetKind::Timer) {
            for &timer in queue {
                match (timer as *const rcl_timer_t)
                    .borrow_impl()
                    .unwrap()
                    .get_time_until_next_call()
                {
                    Ok(diff) => timeout = timeout.min(diff),
                    Err(_) => return Duration::ZERO,
                }
            }
        }
        timeout
    }
}

impl_has_impl_ptr!(rcl_wait_set_t, rcl_wait_set_impl_t, WaitSetImpl);

impl rcl_wait_set_t {
    fn new() -> Self {
        let ws_impl = WaitSetImpl::new();
        let mut ws = Self::default();
        unsafe {
            ws.impl_ = Box::into_raw(Box::new(ws_impl)) as _;
        }
        ws.reset_pointers();
        ws
    }

    fn borrow_mut_impl(&mut self) -> &mut WaitSetImpl {
        unsafe { &mut *(self.impl_ as *mut WaitSetImpl) }
    }

    fn borrow_impl(&self) -> &WaitSetImpl {
        unsafe { &*(self.impl_ as *const WaitSetImpl) }
    }

    // fn resize(&mut self, kind: &WaitSetKind, len: usize) {
    //     // FIXME: we lost the length information in WaitSetImpl
    //     // self.borrow_mut_impl().resize(kind, len);
    //
    //     use WaitSetKind::*;
    //     match kind {
    //         Subscription => {
    //             self.size_of_clients = len;
    //         }
    //         Client => {
    //             self.size_of_clients = len;
    //         }
    //         Service => {
    //             self.size_of_services = len;
    //         }
    //     }
    // }

    fn reset(&mut self) {
        self.borrow_mut_impl().reset_all();
        self.reset_pointers();
    }

    fn reset_pointers(&mut self) {
        self.subscriptions = std::ptr::null_mut();
        self.clients = std::ptr::null_mut();
        self.services = std::ptr::null_mut();
        self.guard_conditions = std::ptr::null_mut();
        self.timers = std::ptr::null_mut();
        self.events = std::ptr::null_mut();
    }

    // fn write_ptr(&mut self, kind: &WaitSetKind, box_ptr: &Box<[*const c_void]>) {
    //     match kind {
    //         WaitSetKind::Subscription => {
    //             self.subscriptions = box_ptr.as_ptr() as _;
    //             self.size_of_subscriptions = box_ptr.len();
    //         }
    //         WaitSetKind::Client => {
    //             self.clients = box_ptr.as_ptr() as _;
    //             self.size_of_clients = box_ptr.len();
    //         }
    //         WaitSetKind::Service => {
    //             self.services = box_ptr.as_ptr() as _;
    //             self.size_of_services = box_ptr.len();
    //
    //             dbg!(self.services);
    //             unsafe {
    //                 let x = std::slice::from_raw_parts_mut(self.services, self.size_of_services);
    //                 dbg!(x);
    //             }
    //         }
    //         WaitSetKind::GuradCondition => {
    //             self.guard_conditions = box_ptr.as_ptr() as _;
    //             self.size_of_guard_conditions = box_ptr.len();
    //         }
    //     }
    // }

    fn write(&mut self, kind: WaitSetKind, ptr: *mut *const c_void, len: usize) {
        match kind {
            WaitSetKind::Subscription => {
                self.subscriptions = ptr as _;
                self.size_of_subscriptions = len;
            }
            WaitSetKind::Client => {
                self.clients = ptr as _;
                self.size_of_clients = len;
            }
            WaitSetKind::Service => {
                self.services = ptr as _;
                self.size_of_services = len;
            }
            WaitSetKind::GuradCondition => {
                self.guard_conditions = ptr as _;
                self.size_of_guard_conditions = len;
            }
            WaitSetKind::Timer => {
                self.timers = ptr as _;
                self.size_of_timers = len;
            }
            WaitSetKind::Event => {
                self.events = ptr as _;
                self.size_of_events = len;
            }
        }
    }

    fn output(&mut self) {
        let mut x: HashMap<_, _> = self
            .borrow_mut_impl()
            .hmap
            .drain()
            .map(|(kind, queue)| {
                let len = queue.len();
                let ptr = Box::into_raw(queue.into_boxed_slice()) as *mut *const c_void;
                (kind, (ptr, len))
            })
            .collect();
        x.drain().for_each(|(kind, (ptr, len))| {
            self.write(kind, ptr, len);
        });
    }

    fn drop_pointer(&mut self, kind: WaitSetKind) {
        match kind {
            WaitSetKind::Subscription => {
                std::mem::drop(unsafe {
                    Box::from_raw(std::slice::from_raw_parts_mut(
                        self.subscriptions,
                        self.size_of_subscriptions,
                    ))
                });
            }
            WaitSetKind::Client => {
                std::mem::drop(unsafe {
                    Box::from_raw(std::slice::from_raw_parts_mut(
                        self.clients,
                        self.size_of_clients,
                    ))
                });
            }
            WaitSetKind::Service => {
                std::mem::drop(unsafe {
                    Box::from_raw(std::slice::from_raw_parts_mut(
                        self.services,
                        self.size_of_services,
                    ))
                });
            }
            WaitSetKind::GuradCondition => {
                std::mem::drop(unsafe {
                    Box::from_raw(std::slice::from_raw_parts_mut(
                        self.guard_conditions,
                        self.size_of_guard_conditions,
                    ))
                });
            }
            WaitSetKind::Timer => {
                std::mem::drop(unsafe {
                    Box::from_raw(std::slice::from_raw_parts_mut(
                        self.timers,
                        self.size_of_events,
                    ))
                });
            }
            WaitSetKind::Event => {
                std::mem::drop(unsafe {
                    Box::from_raw(std::slice::from_raw_parts_mut(
                        self.events,
                        self.size_of_events,
                    ))
                });
            }
        }
    }

    fn reset2(&mut self) {
        WaitSetKind::iter().for_each(|x| {
            self.drop_pointer(x);
        });
    }

    // fn assign_pointers(&mut self) {
    //     dbg!(
    //         self.borrow_impl()
    //             .mirror
    //             .clone()
    //             .as_ref()
    //             .unwrap()
    //             .get(&WaitSetKind::Service)
    //             .unwrap()
    //             .as_ptr()
    //     );
    //     if let Some(mirror) = self.borrow_impl().mirror.clone() {
    //         mirror.iter().for_each(|(kind, box_ptr)| {
    //             tracing::error!("Assign {kind:?}: {box_ptr:?}");
    //             self.write_ptr(kind, box_ptr);
    //         });
    //     }
    //     unsafe {
    //         let x = std::slice::from_raw_parts_mut(self.services, self.size_of_services);
    //         dbg!("finished", x);
    //         dbg!(
    //             self.borrow_impl()
    //                 .mirror
    //                 .as_ref()
    //                 .unwrap()
    //                 .get(&WaitSetKind::Service)
    //                 .unwrap()
    //                 .as_ptr()
    //         );
    //     }
    // }

    // fn debug(&self) {
    //     WaitSetKind::iter().for_each(|kind| match kind {
    //         WaitSetKind::Subscription => unsafe {
    //             let x =
    //                 std::slice::from_raw_parts_mut(self.subscriptions, self.size_of_subscriptions);
    //             dbg!(kind, x);
    //         },
    //         WaitSetKind::Client => unsafe {
    //             let x = std::slice::from_raw_parts_mut(self.clients, self.size_of_clients);
    //             dbg!(kind, x);
    //         },
    //         WaitSetKind::Service => unsafe {
    //             let x = std::slice::from_raw_parts_mut(self.services, self.size_of_services);
    //             dbg!(kind, x);
    //         },
    //         WaitSetKind::GuradCondition => unsafe {
    //             let x = std::slice::from_raw_parts_mut(
    //                 self.guard_conditions,
    //                 self.size_of_guard_conditions,
    //             );
    //             dbg!(kind, x);
    //         },
    //     });
    // }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_wait_set_init(
    wait_set: *mut rcl_wait_set_t,
    number_of_subscriptions: usize,
    number_of_guard_conditions: usize,
    number_of_timers: usize,
    number_of_clients: usize,
    number_of_services: usize,
    number_of_events: usize,
    context: *mut rcl_context_t,
    _allocator: rcl_allocator_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_init");

    if wait_set.is_null() || context.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    if let (Ok(ws_impl), Ok(ctx_impl)) = (wait_set.borrow_mut_impl(), context.borrow_impl()) {
        ws_impl.notifier = Some(ctx_impl.share_notifier());
        ws_impl.allocator = _allocator;
    } else {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // FIXME: we lost the length information in WaitSetImpl
    // let x = wait_set.borrow_mut_impl().unwrap();
    // for (kind, len) in [
    //     (WaitSetKind::Subscription, number_of_subscriptions),
    //     (WaitSetKind::Client, number_of_clients),
    //     (WaitSetKind::Service, number_of_services),
    // ] {
    //     x.resize(&kind, len);
    // }

    unsafe {
        (*wait_set).size_of_subscriptions = number_of_subscriptions;
        (*wait_set).size_of_guard_conditions = number_of_guard_conditions;
        (*wait_set).size_of_timers = number_of_timers;
        (*wait_set).size_of_clients = number_of_clients;
        (*wait_set).size_of_services = number_of_services;
        (*wait_set).size_of_events = number_of_events;
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_fini(wait_set: *mut rcl_wait_set_t) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_fini");
    if wait_set.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if let Ok(impl_) = wait_set.borrow_mut_impl() {
        impl_.reset_all();
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_clear(wait_set: *mut rcl_wait_set_t) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_clear");
    wait_set.borrow_mut_impl().unwrap().reset_all();
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait(wait_set: *mut rcl_wait_set_t, timeout: i64) -> rcl_ret_t {
    tracing::trace!("rcl_wait, timeout={timeout}");
    if wait_set.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let ws_impl = wait_set.borrow_mut_impl().unwrap();

    // -1 => u64::Max
    // =0 => 0 in u64
    // >0 => >0 in u64
    let timeout = std::time::Duration::from_nanos(timeout as u64).min(ws_impl.min_timeout());

    ws_impl.wait(ws_impl.notifier.as_ref().unwrap(), timeout);

    let ret = match ws_impl.check_ready() {
        Ok(ready) => {
            if ready {
                RCL_RET_OK
            } else {
                RCL_RET_TIMEOUT
            }
        }
        Err(err) => {
            tracing::error!(err);
            RCL_RET_ERROR
        }
    };
    ws_impl.reset_guard_conditions();
    unsafe {
        (*wait_set).output();
    }
    ret as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_is_valid(wait_set: *const rcl_wait_set_t) -> bool {
    tracing::trace!("rcl_wait_set_is_valid");
    !wait_set.is_null() && unsafe { !(*wait_set).impl_.is_null() }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_wait_set() -> rcl_wait_set_t {
    tracing::trace!("rcl_get_zero_initialized_wait_set");
    unsafe { std::mem::zeroed() }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_resize(
    wait_set: *mut rcl_wait_set_t,
    subscriptions_size: usize,
    guard_conditions_size: usize,
    timers_size: usize,
    clients_size: usize,
    services_size: usize,
    events_size: usize,
) -> rcl_ret_t {
    tracing::trace!(
        subscriptions_size,
        guard_conditions_size,
        timers_size,
        clients_size,
        services_size,
        events_size,
        "rcl_wait_set_resize called with sizes"
    );

    if wait_set.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    unsafe {
        (*wait_set).size_of_subscriptions = subscriptions_size;
        (*wait_set).size_of_guard_conditions = guard_conditions_size;
        (*wait_set).size_of_timers = timers_size;
        (*wait_set).size_of_clients = clients_size;
        (*wait_set).size_of_services = services_size;
        (*wait_set).size_of_events = events_size;
    }

    RCL_RET_OK as _
}

macro_rules! impl_wait_set_add {
    ($fn_name:ident, $ty:ty, $kind:path) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_name(
            wait_set: *mut rcl_wait_set_t,
            item: *const $ty,
            index: *mut usize,
        ) -> rcl_ret_t {
            tracing::trace!(stringify!($fn_name));
            assert!(!item.is_null());

            let x = wait_set.borrow_mut_impl().unwrap();
            x.insert(&$kind, item as _);
            if !index.is_null() {
                unsafe {
                    *index = x.queue_len(&$kind) - 1;
                }
            }
            RCL_RET_OK as _
        }
    };

    ($fn_name:ident, $ty:ty, $kind:path, null) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_name(
            wait_set: *mut rcl_wait_set_t,
            item: *const $ty,
            index: *mut usize,
        ) -> rcl_ret_t {
            tracing::trace!(stringify!($fn_name));
            assert!(!item.is_null());

            let x = wait_set.borrow_mut_impl().unwrap();
            x.insert(&$kind, std::ptr::null());
            if !index.is_null() {
                unsafe {
                    *index = x.queue_len(&$kind) - 1;
                }
            }
            RCL_RET_OK as _
        }
    };
}

impl_wait_set_add!(
    rcl_wait_set_add_subscription,
    rcl_subscription_t,
    WaitSetKind::Subscription
);
impl_wait_set_add!(rcl_wait_set_add_client, rcl_client_t, WaitSetKind::Client);
impl_wait_set_add!(
    rcl_wait_set_add_service,
    rcl_service_t,
    WaitSetKind::Service
);
impl_wait_set_add!(
    rcl_wait_set_add_guard_condition,
    rcl_guard_condition_t,
    WaitSetKind::GuradCondition
);
impl_wait_set_add!(rcl_wait_set_add_timer, rcl_timer_t, WaitSetKind::Timer);
impl_wait_set_add!(
    rcl_wait_set_add_event,
    rcl_event_t,
    WaitSetKind::Event,
    null
);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_get_allocator(
    wait_set: *const rcl_wait_set_t,
    allocator: *mut rcl_allocator_t,
) -> rcl_ret_t {
    if wait_set.is_null() || allocator.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }
    if !rcl_wait_set_is_valid(wait_set) {
        return RCL_RET_WAIT_SET_INVALID as _;
    }
    unsafe {
        *allocator = wait_set.borrow_impl().unwrap().allocator;
    }
    RCL_RET_OK as _
}
