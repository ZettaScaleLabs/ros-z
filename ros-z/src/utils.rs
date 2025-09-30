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
    (ZPubBuilder<$t:ident, $s:ident>) => {
        impl<$t, $s> ZPubBuilder<$t, $s>
        where
            $t: crate::msg::ZMessage,
            $s: for<'a> crate::msg::ZSerializer<Input<'a> = &'a $t>,
        {
            pub fn with_type_info(mut self, type_info: crate::entity::TypeInfo) -> Self {
                self.entity.type_info = Some(type_info);
                self
            }
        }
    };
    (ZSubBuilder<$t:ident, $s:ident>) => {
        impl<$t, $s> ZSubBuilder<$t, $s>
        where
            $t: crate::msg::ZMessage,
            $s: crate::msg::ZDeserializer,
        {
            pub fn with_type_info(mut self, type_info: crate::entity::TypeInfo) -> Self {
                self.entity.type_info = Some(type_info);
                self
            }
        }
    };
}
