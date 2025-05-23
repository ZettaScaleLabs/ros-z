#![allow(unused)]

use crate::pubsub::SubscriptionImpl;
use crate::ros::*;

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_init(
    wait_set: *mut rcl_wait_set_t,
    number_of_subscriptions: usize,
    _number_of_guard_conditions: usize,
    _number_of_timers: usize,
    _number_of_clients: usize,
    _number_of_services: usize,
    _number_of_events: usize,
    _context: *mut rcl_context_t,
    _allocator: rcl_allocator_t,
) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_init");

    let subs =
        vec![std::ptr::null::<rcl_subscription_t>(); number_of_subscriptions].into_boxed_slice();
    unsafe {
        (*wait_set).subscriptions = Box::into_raw(subs) as *mut *const rcl_subscription_t;
        (*wait_set).size_of_subscriptions = number_of_subscriptions;
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_fini(wait_set: *mut rcl_wait_set_t) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_fini");
    unsafe {
        // FIXME: This causes memory leak
        let slice = std::slice::from_raw_parts_mut(
            (*wait_set).subscriptions,
            (*wait_set).size_of_subscriptions,
        );
        let boxed = Box::from_raw(slice);

        // Dropping the box will free the memory
        drop(boxed);
    }
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait_set_clear(_wait_set: *mut rcl_wait_set_t) -> rcl_ret_t {
    tracing::trace!("rcl_wait_set_clear");
    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_wait(wait_set: *mut rcl_wait_set_t, timeout: i64) -> rcl_ret_t {
    tracing::trace!("rcl_wait");
    if wait_set.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let timeout = std::time::Duration::from_nanos(timeout as u64);

    unsafe {
        for i in 0..(*wait_set).size_of_subscriptions {
            let sub_ptr = *(*wait_set).subscriptions.add(i);
            if sub_ptr.is_null() {
                continue;
            }

            let sub_impl = &mut *((*sub_ptr).impl_ as *mut SubscriptionImpl);

            if !sub_impl.zsub.queue.is_empty() {
                return RCL_RET_OK as _;
            }

            let (lock, cv) = &*sub_impl.cv;
            let mut started = lock.lock();
            if sub_impl.zsub.queue.len() == 0 && !*started {
                cv.wait_for(&mut started, timeout);
            }
        }
    }

    RCL_RET_OK as _
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_wait_set() -> rcl_wait_set_t {
    rcl_wait_set_t::default()
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
    tracing::error!(
        subscriptions_size,
        guard_conditions_size,
        timers_size,
        clients_size,
        services_size,
        events_size,
        "rcl_wait_set_resize called with sizes"
    );

    macro_rules! init_wait_set_fields {
        ($wait_set:expr; $(($field:ident, $size_field:ident, $ty:ty, $len:expr)),+ $(,)?) => {{
            $(
                let len = $len;
                (*$wait_set).$field = Box::into_raw(
                    vec![std::ptr::null::<$ty>(); len].into_boxed_slice()
                ) as *mut *const _;
                (*$wait_set).$size_field = len;
            )+
        }};
    }

    // TODO: fix this
    unsafe {
        init_wait_set_fields!(wait_set;
            (subscriptions, size_of_subscriptions, rcl_subscription_t, subscriptions_size),
            (guard_conditions, size_of_guard_conditions, rcl_guard_condition_t, guard_conditions_size),
            (timers, size_of_timers, rcl_timer_t, timers_size),
            (clients, size_of_clients, rcl_client_t, clients_size),
            (services, size_of_services, rcl_service_t, services_size),
            (events, size_of_events, rcl_event_t, events_size),
        );
    }

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
                let slice = std::slice::from_raw_parts_mut(
                    (*wait_set).$field,
                    (*wait_set).$size_field,
                );
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

impl_wait_set_add_fn!(rcl_wait_set_add_subscription, subscriptions, size_of_subscriptions, rcl_subscription_t);
impl_wait_set_add_fn!(rcl_wait_set_add_service, services, size_of_services, rcl_service_t);
impl_wait_set_add_fn!(rcl_wait_set_add_event, events, size_of_events, rcl_event_t);
impl_wait_set_add_fn!(rcl_wait_set_add_guard_condition, guard_conditions, size_of_guard_conditions, rcl_guard_condition_t);
impl_wait_set_add_fn!(rcl_wait_set_add_client, clients, size_of_clients, rcl_client_t);
impl_wait_set_add_fn!(rcl_wait_set_add_timer, timers, size_of_timers, rcl_timer_t);

// #[unsafe(no_mangle)]
// pub extern "C" fn rcl_wait_set_add_subscription(
//     wait_set: *mut rcl_wait_set_t,
//     subscription: *const rcl_subscription_t,
//     index: *mut usize,
// ) -> rcl_ret_t {
//     tracing::trace!("rcl_wait_set_add_subscription");
//     unsafe {
//         let slice = std::slice::from_raw_parts_mut(
//             (*wait_set).subscriptions,
//             (*wait_set).size_of_subscriptions,
//         );
//         for (i, slot) in slice.iter_mut().enumerate() {
//             if slot.is_null() {
//                 *slot = subscription;
//                 if !index.is_null() {
//                     *index = i;
//                 }
//                 return RCL_RET_OK as _;
//             }
//         }
//     }
//     RCL_RET_ERROR as _
// }
