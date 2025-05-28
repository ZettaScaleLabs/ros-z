#![allow(unused)]

use std::collections::VecDeque;

use crate::pubsub::SubscriptionImpl;
use crate::{
    c_void, impl_has_impl_ptr,
    ros::*,
    traits::{BorrowImpl, OwnImpl},
};

#[derive(Debug, Default)]
pub struct WaitSetImpl {
    subscriptions: VecDeque<*const c_void>,
    clients: VecDeque<*const c_void>,
    services: VecDeque<*const c_void>,
}

impl_has_impl_ptr!(rcl_wait_set_t, rcl_wait_set_impl_t, WaitSetImpl);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_init(
    wait_set: *mut rcl_wait_set_t,
    number_of_subscriptions: usize,
    number_of_guard_conditions: usize,
    number_of_timers: usize,
    number_of_clients: usize,
    number_of_services: usize,
    number_of_events: usize,
    _context: *mut rcl_context_t,
    _allocator: rcl_allocator_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_init");

    // FIXME: we should initiate it properly.
    rcl_wait_set_resize(
        wait_set,
        number_of_subscriptions,
        number_of_guard_conditions,
        number_of_timers,
        number_of_clients,
        number_of_services,
        number_of_events,
    );

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_fini(wait_set: *mut rcl_wait_set_t) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_fini");

    let x = wait_set.borrow_mut_impl().unwrap();
    x.services = VecDeque::new();
    x.subscriptions = VecDeque::new();
    x.clients = VecDeque::new();
    // unsafe {
    //     let x = Box::from_raw((*wait_set).impl_ as *mut WaitSetImpl);
    //     std::mem::drop(x);
    // }

    macro_rules! drop_each {
        ($(($field:ident, $len:ident)),+ $(,)?) => {
            unsafe {
                $(
                    let slice = std::slice::from_raw_parts_mut(
                        (*wait_set).$field,
                        (*wait_set).$len,
                    );
                    std::mem::drop(Box::from_raw(slice));
                )+
            }
        };
    }

    drop_each!(
        (subscriptions, size_of_subscriptions),
        (guard_conditions, size_of_guard_conditions),
        (timers, size_of_timers),
        (clients, size_of_clients),
        (services, size_of_services),
        (events, size_of_events),
    );
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_clear(wait_set: *mut rcl_wait_set_t) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_clear");

    macro_rules! nullify_each {
        ($(($field:ident, $len:ident)),+ $(,)?) => {
            unsafe {
                $(
                    let skip = (*wait_set).$field.is_null() || (*wait_set).$len == 0;
                    if (!skip) {
                        let slice = std::slice::from_raw_parts_mut(
                            (*wait_set).$field,
                            (*wait_set).$len,
                        );
                        slice.iter_mut().for_each(|x| *x = std::ptr::null());
                    }
                )+
            }
        };
    }

    nullify_each!(
        (subscriptions, size_of_subscriptions),
        (guard_conditions, size_of_guard_conditions),
        (timers, size_of_timers),
        (clients, size_of_clients),
        (services, size_of_services),
        (events, size_of_events),
    );

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait(wait_set: *mut rcl_wait_set_t, timeout: i64) -> rcl_ret_t {
    tracing::trace!("rcl_wait");
    if wait_set.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let timeout = std::time::Duration::from_nanos(timeout as u64);
    let ws = wait_set.borrow_mut_impl().unwrap();

    macro_rules! wait_for_each {
        ($(($field:ident, $ty:ty)),+ $(,)?) => {
            $(
                for (idx, ptr ) in ws.$field.drain(..).enumerate() {
                    let ptr = ptr as *const $ty;
                    if ptr.borrow_impl().unwrap().wait(timeout) {
                        unsafe {
                            *(*wait_set).$field.add(idx) = ptr;
                        }
                    }
                }
            )+
        };
    }

    wait_for_each! {
        (subscriptions, rcl_subscription_t),
        (clients, rcl_client_t),
        (services, rcl_service_t),
    }

    // for (idx, ptr ) in ws.subscriptions.drain(..).enumerate() {
    //     let ptr = ptr as *const rcl_subscription_t;
    //     if ptr.borrow_impl().unwrap().wait(timeout) {
    //         unsafe {
    //             *(*wait_set).subscriptions.add(idx) = ptr;
    //         }
    //     }
    // }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_wait_set() -> rcl_wait_set_t {
    let mut wait_set = rcl_wait_set_t::default();

    let x = WaitSetImpl::default();
    unsafe {
        wait_set.impl_ = Box::into_raw(Box::new(x)) as _;
    }

    rcl_wait_set_resize(&mut wait_set, 0, 0, 0, 0, 0, 0);
    wait_set
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

    macro_rules! init_wait_set_fields {
        ($(($field:ident, $size_field:ident, $ty:ty, $len:expr)),+ $(,)?) => {
            unsafe {
                $(
                    (*wait_set).$field = Box::into_raw(
                        vec![std::ptr::null::<$ty>(); $len].into_boxed_slice()
                    ) as *mut *const _;
                    (*wait_set).$size_field = $len;
                )+
            }
        };
    }

    // unsafe {
    //     let x = &mut *((*wait_set).impl_ as *mut WaitSetImpl);
    //     x.subscriptions.resize(subscriptions_size, std::ptr::null());
    //     x.clients.resize(clients_size, std::ptr::null());
    //     x.services.resize(services_size, std::ptr::null());
    // }

    // FIXME: memory leak
    init_wait_set_fields!(
        (
            subscriptions,
            size_of_subscriptions,
            rcl_subscription_t,
            subscriptions_size
        ),
        (
            guard_conditions,
            size_of_guard_conditions,
            rcl_guard_condition_t,
            guard_conditions_size
        ),
        (timers, size_of_timers, rcl_timer_t, timers_size),
        (clients, size_of_clients, rcl_client_t, clients_size),
        (services, size_of_services, rcl_service_t, services_size),
        (events, size_of_events, rcl_event_t, events_size),
    );

    RCL_RET_OK as _
}

macro_rules! impl_wait_set_add_fn {
    ($fn_name:ident, $field:ident, $size_field:ident, $ty:ty) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_name(
            wait_set: *mut rcl_wait_set_t,
            item: *const $ty,
            index: *mut usize,
        ) -> rcl_ret_t {
            tracing::trace!(stringify!($fn_name));
            unsafe {
                let slice =
                    std::slice::from_raw_parts_mut((*wait_set).$field, (*wait_set).$size_field);
                for (i, slot) in slice.iter_mut().enumerate() {
                    if slot.is_null() {
                        *slot = item;
                        if !index.is_null() {
                            *index = i;
                        }
                        return RCL_RET_OK as _;
                    }
                }
            }
            RCL_RET_ERROR as _
        }
    };
}

// impl_wait_set_add_fn!(rcl_wait_set_add_subscription, subscriptions, size_of_subscriptions, rcl_subscription_t);
// impl_wait_set_add_fn!(rcl_wait_set_add_client, clients, size_of_clients, rcl_client_t);
// impl_wait_set_add_fn!(rcl_wait_set_add_service, services, size_of_services, rcl_service_t);
impl_wait_set_add_fn!(rcl_wait_set_add_event, events, size_of_events, rcl_event_t);
impl_wait_set_add_fn!(
    rcl_wait_set_add_guard_condition,
    guard_conditions,
    size_of_guard_conditions,
    rcl_guard_condition_t
);
impl_wait_set_add_fn!(rcl_wait_set_add_timer, timers, size_of_timers, rcl_timer_t);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_add_subscription(
    wait_set: *mut rcl_wait_set_t,
    item: *const rcl_subscription_t,
    index: *mut usize,
) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_add_subscription");

    assert!(!item.is_null());

    let x = wait_set.borrow_mut_impl().unwrap();
    x.subscriptions.push_back(item as _);

    // NOTE: index could be a nullptr when the user doesn't care the index ¯\_(ツ)_/¯
    if !index.is_null() {
        unsafe {
            *index = x.subscriptions.len();
        }
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_add_client(
    wait_set: *mut rcl_wait_set_t,
    item: *const rcl_client_t,
    index: *mut usize,
) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_add_client");

    assert!(!item.is_null());

    let x = wait_set.borrow_mut_impl().unwrap();
    x.clients.push_back(item as _);

    if !index.is_null() {
        unsafe {
            *index = x.clients.len();
        }
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_add_service(
    wait_set: *mut rcl_wait_set_t,
    item: *const rcl_service_t,
    index: *mut usize,
) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_add_service");

    assert!(!item.is_null());

    let x = wait_set.borrow_mut_impl().unwrap();
    x.services.push_back(item as _);

    if !index.is_null() {
        unsafe {
            *index = x.services.len();
        }
    }
    RCL_RET_OK as _
}
