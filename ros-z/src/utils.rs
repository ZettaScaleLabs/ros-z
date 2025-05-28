#[macro_export]
macro_rules! impl_with_type_info {
    ($type:ident<$t:ident>) => {
        impl<$t> $type<$t> {
            pub fn with_type_info(mut self, type_info: crate::entity::TypeInfo) -> Self {
                self.entity.type_info = Some(type_info);
                self
            }
        }
    };
}
