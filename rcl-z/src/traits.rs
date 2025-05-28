use std::fmt::Display;

pub trait HasImplPtr {
    type ImplType;
    type CImplType;
    fn get_impl(&self) -> *mut Self::CImplType;
    fn get_mut_impl(&mut self) -> &mut *mut Self::CImplType;
}

#[derive(Debug, Clone, Copy)]
pub enum ImplAccessError {
    NullSelfPtr,
    NullImplPtr,
    NonNullImplPtr,
}

impl Display for ImplAccessError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let ctx = match self {
            ImplAccessError::NullSelfPtr => "Null self pointer",
            ImplAccessError::NullImplPtr => "Null impl pointer",
            ImplAccessError::NonNullImplPtr => "Non-null impl pointer",
        };
        write!(f, "{ctx} detected")
    }
}

impl std::error::Error for ImplAccessError {}

pub trait BorrowImpl<T: HasImplPtr> {
    fn borrow_impl<'a>(self) -> Result<&'a T::ImplType, ImplAccessError>;
}

pub trait OwnImpl<T: HasImplPtr> {
    fn own_impl(self) -> Result<Box<T::ImplType>, ImplAccessError>;
    fn assign_impl(self, val: T::ImplType) -> Result<(), ImplAccessError>;
    fn borrow_mut_impl<'a>(self) -> Result<&'a mut T::ImplType, ImplAccessError>;
}

impl<T> BorrowImpl<T> for *const T
where
    T: HasImplPtr,
{
    fn borrow_impl<'a>(self) -> Result<&'a T::ImplType, ImplAccessError> {
        if self.is_null() {
            return Err(ImplAccessError::NullSelfPtr);
        }
        unsafe {
            let impl_ptr = (*self).get_impl();
            if impl_ptr.is_null() {
                return Err(ImplAccessError::NullImplPtr);
            }
            Ok(&*(impl_ptr as *const _))
        }
    }
}

impl<T> OwnImpl<T> for *mut T
where
    T: HasImplPtr,
{
    fn borrow_mut_impl<'a>(self) -> Result<&'a mut T::ImplType, ImplAccessError> {
        if self.is_null() {
            return Err(ImplAccessError::NullSelfPtr);
        }
        unsafe {
            let impl_ptr = (*self).get_impl();
            if impl_ptr.is_null() {
                return Err(ImplAccessError::NullImplPtr);
            }
            Ok(&mut *(impl_ptr as *mut _))
        }
    }

    fn own_impl(self) -> Result<Box<T::ImplType>, ImplAccessError> {
        if self.is_null() {
            return Err(ImplAccessError::NullSelfPtr);
        }
        unsafe {
            let impl_ptr = (*self).get_impl();
            if impl_ptr.is_null() {
                return Err(ImplAccessError::NullImplPtr);
            }
            let owned = Box::from_raw(impl_ptr as _);
            *(*self).get_mut_impl() = std::ptr::null_mut();
            Ok(owned)
        }
    }

    fn assign_impl(self, val: T::ImplType) -> Result<(), ImplAccessError> {
        if self.is_null() {
            return Err(ImplAccessError::NullSelfPtr);
        }
        unsafe {
            let mut_impl_ptr = (*self).get_mut_impl();
            if !mut_impl_ptr.is_null() {
                return Err(ImplAccessError::NonNullImplPtr);
            }
            *mut_impl_ptr = Box::into_raw(Box::new(val)) as _;
            Ok(())
        }
    }
}

#[macro_export]
macro_rules! impl_has_impl_ptr {
    ($ctype:ty, $cimpl_type:ty, $impl_type:ty) => {
        impl crate::traits::HasImplPtr for $ctype {
            type ImplType = $impl_type;
            type CImplType = $cimpl_type;
            fn get_impl(&self) -> *mut Self::CImplType {
                self.impl_
            }
            fn get_mut_impl(&mut self) -> &mut *mut Self::CImplType {
                &mut self.impl_
            }
        }
    };
}
