//! Derive macros for Python message bridge traits
//!
//! Provides `FromPyMessage` and `IntoPyMessage` derive macros for automatic
//! conversion between Python msgspec structs and Rust ROS message types.

#![allow(clippy::collapsible_if)]

use proc_macro::TokenStream;
use quote::quote;
use syn::{
    Attribute, Data, DeriveInput, Fields, GenericArgument, Ident, Lit, Meta, PathArguments, Type,
    parse_macro_input,
};

/// Derive macro for extracting Rust messages from Python objects
///
/// # Example
/// ```ignore
/// #[derive(FromPyMessage)]
/// #[ros_msg(module = "ros_z_msgs_py.types.std_msgs")]
/// pub struct String {
///     pub data: std::string::String,
/// }
/// ```
#[proc_macro_derive(FromPyMessage, attributes(ros_msg))]
pub fn derive_from_py_message(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match impl_from_py_message(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

/// Derive macro for constructing Python objects from Rust messages
///
/// # Example
/// ```ignore
/// #[derive(IntoPyMessage)]
/// #[ros_msg(module = "ros_z_msgs_py.types.std_msgs")]
/// pub struct String {
///     pub data: std::string::String,
/// }
/// ```
#[proc_macro_derive(IntoPyMessage, attributes(ros_msg))]
pub fn derive_into_py_message(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match impl_into_py_message(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

fn impl_from_py_message(input: &DeriveInput) -> syn::Result<proc_macro2::TokenStream> {
    let name = &input.ident;

    let Data::Struct(ref data) = input.data else {
        return Err(syn::Error::new_spanned(
            input,
            "FromPyMessage only supports structs",
        ));
    };

    let Fields::Named(ref fields) = data.fields else {
        return Err(syn::Error::new_spanned(
            input,
            "FromPyMessage requires named fields",
        ));
    };

    let field_extractions: Vec<proc_macro2::TokenStream> = fields
        .named
        .iter()
        .map(|f| {
            let field_name = f.ident.as_ref().unwrap();
            let field_name_str = field_name_to_py_attr(field_name);
            let field_type = &f.ty;

            // Check for #[ros_msg(zbuf)] attribute
            let use_zbuf = has_ros_msg_attr(&f.attrs, "zbuf");

            generate_field_extraction(field_name, &field_name_str, field_type, use_zbuf)
        })
        .collect::<syn::Result<Vec<_>>>()?;

    Ok(quote! {
        impl ::ros_z::python_bridge::FromPyMessage for #name {
            fn from_py(obj: &::pyo3::Bound<'_, ::pyo3::PyAny>) -> ::pyo3::PyResult<Self> {
                use ::pyo3::types::PyAnyMethods;
                Ok(Self {
                    #(#field_extractions),*
                })
            }
        }
    })
}

fn impl_into_py_message(input: &DeriveInput) -> syn::Result<proc_macro2::TokenStream> {
    let name = &input.ident;

    let Data::Struct(ref data) = input.data else {
        return Err(syn::Error::new_spanned(
            input,
            "IntoPyMessage only supports structs",
        ));
    };

    let Fields::Named(ref fields) = data.fields else {
        return Err(syn::Error::new_spanned(
            input,
            "IntoPyMessage requires named fields",
        ));
    };

    // Extract module path from #[ros_msg(module = "...")] attribute
    let module_path = extract_module_path(&input.attrs)?;

    let field_constructions: Vec<proc_macro2::TokenStream> = fields
        .named
        .iter()
        .map(|f| {
            let field_name = f.ident.as_ref().unwrap();
            let field_name_str = field_name_to_py_attr(field_name);
            let field_type = &f.ty;

            // Check for #[ros_msg(zbuf)] attribute
            let use_zbuf = has_ros_msg_attr(&f.attrs, "zbuf");

            generate_field_construction(field_name, &field_name_str, field_type, use_zbuf)
        })
        .collect::<syn::Result<Vec<_>>>()?;

    let name_str = name.to_string();

    Ok(quote! {
        impl ::ros_z::python_bridge::IntoPyMessage for #name {
            fn into_py_message(&self, py: ::pyo3::Python) -> ::pyo3::PyResult<::pyo3::PyObject> {
                use ::pyo3::types::{PyAnyMethods, PyDictMethods, PyModuleMethods};
                let module = ::pyo3::types::PyModule::import_bound(py, #module_path)?;
                let class = module.getattr(#name_str)?;

                let kwargs = ::pyo3::types::PyDict::new_bound(py);
                #(#field_constructions)*

                class.call((), Some(&kwargs)).map(|obj| obj.into())
            }
        }
    })
}

/// Generate extraction code for a single field
fn generate_field_extraction(
    field_name: &Ident,
    field_name_str: &str,
    field_type: &Type,
    use_zbuf: bool,
) -> syn::Result<proc_macro2::TokenStream> {
    // Handle ZBuf fields specially - try zero-copy paths first
    if use_zbuf {
        return Ok(quote! {
            #field_name: {
                use ::pyo3::types::{PyBytesMethods, PyByteArrayMethods};
                let py_attr = obj.getattr(#field_name_str)?;
                // Try ZBufView first - clone is cheap (ref-counted ZSlices)
                if let Ok(view) = py_attr.downcast::<::ros_z::zbuf_view::ZBufView>() {
                    view.borrow().zbuf().clone()
                } else if let Ok(bytes) = py_attr.downcast::<::pyo3::types::PyBytes>() {
                    ::ros_z::ZBuf::from(bytes.as_bytes().to_vec())
                } else if let Ok(bytearray) = py_attr.downcast::<::pyo3::types::PyByteArray>() {
                    // SAFETY: We immediately copy the data
                    ::ros_z::ZBuf::from(unsafe { bytearray.as_bytes() }.to_vec())
                } else {
                    // Fallback for lists (slow path)
                    let bytes: Vec<u8> = py_attr.extract()?;
                    ::ros_z::ZBuf::from(bytes)
                }
            }
        });
    }

    // Analyze the type
    match classify_type(field_type) {
        TypeClass::Primitive => Ok(quote! {
            #field_name: obj.getattr(#field_name_str)?.extract()?
        }),

        TypeClass::String => Ok(quote! {
            #field_name: obj.getattr(#field_name_str)?.extract()?
        }),

        TypeClass::Vec(inner) => {
            let inner_class = classify_type(&inner);
            match inner_class {
                // Special case for Vec<u8> - use buffer protocol for performance
                TypeClass::Primitive if is_u8_type(&inner) => Ok(quote! {
                    #field_name: {
                        use ::pyo3::types::{PyBytesMethods, PyByteArrayMethods};
                        let py_attr = obj.getattr(#field_name_str)?;
                        // Try bytes/bytearray first (fast path - buffer protocol)
                        if let Ok(bytes) = py_attr.downcast::<::pyo3::types::PyBytes>() {
                            bytes.as_bytes().to_vec()
                        } else if let Ok(bytearray) = py_attr.downcast::<::pyo3::types::PyByteArray>() {
                            // SAFETY: We immediately copy the data
                            unsafe { bytearray.as_bytes() }.to_vec()
                        } else {
                            // Fallback for lists (slow path)
                            py_attr.extract()?
                        }
                    }
                }),
                TypeClass::Primitive | TypeClass::String => Ok(quote! {
                    #field_name: obj.getattr(#field_name_str)?.extract()?
                }),
                _ => {
                    // Vec of nested messages - recursively extract
                    Ok(quote! {
                        #field_name: {
                            use ::pyo3::types::PyListMethods;
                            let py_list = obj.getattr(#field_name_str)?;
                            let mut vec = Vec::new();
                            for item in py_list.iter()? {
                                vec.push(<#inner as ::ros_z::python_bridge::FromPyMessage>::from_py(&item?)?);
                            }
                            vec
                        }
                    })
                }
            }
        }

        TypeClass::Array(inner, size) => {
            let inner_class = classify_type(&inner);
            match inner_class {
                TypeClass::Primitive | TypeClass::String => {
                    // For primitive arrays, use zeroed memory for initialization
                    // This works for any array size
                    Ok(quote! {
                        #field_name: {
                            let v: Vec<_> = obj.getattr(#field_name_str)?.extract()?;
                            // Use zeroed memory for arrays larger than 32 elements
                            // SAFETY: All primitive numeric types and bool are valid when zeroed
                            let mut arr: #field_type = unsafe { ::std::mem::zeroed() };
                            let len = ::std::cmp::min(v.len(), #size);
                            arr[..len].copy_from_slice(&v[..len]);
                            arr
                        }
                    })
                }
                _ => {
                    // Fixed array of nested messages
                    Ok(quote! {
                        #field_name: {
                            use ::pyo3::types::PyListMethods;
                            let py_list = obj.getattr(#field_name_str)?;
                            let mut arr: #field_type = ::std::array::from_fn(|_| Default::default());
                            for (i, item) in py_list.iter()?.enumerate().take(#size) {
                                arr[i] = <#inner as ::ros_z::python_bridge::FromPyMessage>::from_py(&item?)?;
                            }
                            arr
                        }
                    })
                }
            }
        }

        TypeClass::Nested => {
            // Nested message - check for None and use Default if None
            Ok(quote! {
                #field_name: {
                    let py_attr = obj.getattr(#field_name_str)?;
                    if py_attr.is_none() {
                        Default::default()
                    } else {
                        <#field_type as ::ros_z::python_bridge::FromPyMessage>::from_py(&py_attr)?
                    }
                }
            })
        }

        TypeClass::ZBuf => Ok(quote! {
            #field_name: {
                use ::pyo3::types::{PyBytesMethods, PyByteArrayMethods};
                let py_attr = obj.getattr(#field_name_str)?;
                // Try bytes/bytearray first (fast path - buffer protocol)
                let bytes: Vec<u8> = if let Ok(bytes) = py_attr.downcast::<::pyo3::types::PyBytes>() {
                    bytes.as_bytes().to_vec()
                } else if let Ok(bytearray) = py_attr.downcast::<::pyo3::types::PyByteArray>() {
                    // SAFETY: We immediately copy the data
                    unsafe { bytearray.as_bytes() }.to_vec()
                } else {
                    // Fallback for lists (slow path)
                    py_attr.extract()?
                };
                ::ros_z::ZBuf::from(bytes)
            }
        }),
    }
}

/// Generate construction code for a single field (Rust -> Python)
fn generate_field_construction(
    field_name: &Ident,
    field_name_str: &str,
    field_type: &Type,
    use_zbuf: bool,
) -> syn::Result<proc_macro2::TokenStream> {
    // Handle ZBuf fields specially - create zero-copy view using buffer protocol
    if use_zbuf {
        return Ok(quote! {
            {
                // Create a ZBufView which implements buffer protocol for zero-copy access
                // Python can use memoryview(zbuf_view) to get zero-copy access to the data
                let zbuf_view = ::ros_z::zbuf_view::ZBufView::new(self.#field_name.clone());
                let py_view = ::pyo3::Py::new(py, zbuf_view)?;
                kwargs.set_item(#field_name_str, py_view)?;
            }
        });
    }

    match classify_type(field_type) {
        TypeClass::Primitive => Ok(quote! {
            kwargs.set_item(#field_name_str, self.#field_name)?;
        }),

        TypeClass::String => Ok(quote! {
            kwargs.set_item(#field_name_str, &self.#field_name)?;
        }),

        TypeClass::Vec(inner) => {
            let inner_class = classify_type(&inner);
            match inner_class {
                // Special case for Vec<u8> - output as bytes for performance
                TypeClass::Primitive if is_u8_type(&inner) => Ok(quote! {
                    {
                        let py_bytes = ::pyo3::types::PyBytes::new_bound(py, &self.#field_name);
                        kwargs.set_item(#field_name_str, py_bytes)?;
                    }
                }),
                TypeClass::Primitive | TypeClass::String => Ok(quote! {
                    kwargs.set_item(#field_name_str, &self.#field_name)?;
                }),
                _ => {
                    // Vec of nested messages
                    Ok(quote! {
                        {
                            use ::pyo3::types::PyListMethods;
                            let py_list = ::pyo3::types::PyList::empty_bound(py);
                            for item in &self.#field_name {
                                py_list.append(
                                    <#inner as ::ros_z::python_bridge::IntoPyMessage>::into_py_message(item, py)?
                                )?;
                            }
                            kwargs.set_item(#field_name_str, py_list)?;
                        }
                    })
                }
            }
        }

        TypeClass::Array(inner, _) => {
            let inner_class = classify_type(&inner);
            match inner_class {
                TypeClass::Primitive | TypeClass::String => Ok(quote! {
                    kwargs.set_item(#field_name_str, self.#field_name.to_vec())?;
                }),
                _ => {
                    // Array of nested messages
                    Ok(quote! {
                        {
                            use ::pyo3::types::PyListMethods;
                            let py_list = ::pyo3::types::PyList::empty_bound(py);
                            for item in &self.#field_name {
                                py_list.append(
                                    <#inner as ::ros_z::python_bridge::IntoPyMessage>::into_py_message(item, py)?
                                )?;
                            }
                            kwargs.set_item(#field_name_str, py_list)?;
                        }
                    })
                }
            }
        }

        TypeClass::Nested => {
            // Nested message - convert using trait
            Ok(quote! {
                kwargs.set_item(
                    #field_name_str,
                    <#field_type as ::ros_z::python_bridge::IntoPyMessage>::into_py_message(&self.#field_name, py)?
                )?;
            })
        }

        TypeClass::ZBuf => Ok(quote! {
            {
                use ::zenoh_buffers::buffer::SplitBuffer;
                let bytes = self.#field_name.contiguous();
                let py_bytes = ::pyo3::types::PyBytes::new_bound(py, bytes.as_ref());
                kwargs.set_item(#field_name_str, py_bytes)?;
            }
        }),
    }
}

/// Type classification for code generation
#[derive(Debug)]
enum TypeClass {
    Primitive,
    String,
    Vec(Box<Type>),
    Array(Box<Type>, usize),
    Nested,
    ZBuf,
}

/// Classify a type for code generation purposes
fn classify_type(ty: &Type) -> TypeClass {
    if let Type::Path(type_path) = ty {
        let segments = &type_path.path.segments;
        if let Some(last_segment) = segments.last() {
            let ident = &last_segment.ident;
            let ident_str = ident.to_string();

            // Check for primitives
            if matches!(
                ident_str.as_str(),
                "bool"
                    | "i8"
                    | "u8"
                    | "i16"
                    | "u16"
                    | "i32"
                    | "u32"
                    | "i64"
                    | "u64"
                    | "f32"
                    | "f64"
            ) {
                return TypeClass::Primitive;
            }

            // Check for String
            if ident_str == "String" {
                return TypeClass::String;
            }

            // Check for ZBuf
            if ident_str == "ZBuf" {
                return TypeClass::ZBuf;
            }

            // Check for Vec
            if ident_str == "Vec" {
                if let PathArguments::AngleBracketed(args) = &last_segment.arguments {
                    if let Some(GenericArgument::Type(inner)) = args.args.first() {
                        return TypeClass::Vec(Box::new(inner.clone()));
                    }
                }
            }
        }
    }

    // Check for arrays
    if let Type::Array(arr) = ty {
        if let syn::Expr::Lit(lit) = &arr.len {
            if let Lit::Int(int_lit) = &lit.lit {
                if let Ok(size) = int_lit.base10_parse::<usize>() {
                    return TypeClass::Array(Box::new((*arr.elem).clone()), size);
                }
            }
        }
    }

    // Default to nested message
    TypeClass::Nested
}

/// Check if a field has a specific ros_msg attribute
fn has_ros_msg_attr(attrs: &[Attribute], attr_name: &str) -> bool {
    for attr in attrs {
        if attr.path().is_ident("ros_msg") {
            if let Ok(meta) = attr.parse_args::<Ident>() {
                if meta == attr_name {
                    return true;
                }
            }
        }
    }
    false
}

/// Extract module path from #[ros_msg(module = "...")] attribute
fn extract_module_path(attrs: &[Attribute]) -> syn::Result<String> {
    for attr in attrs {
        if attr.path().is_ident("ros_msg") {
            if let Ok(Meta::NameValue(nv)) = attr.parse_args() {
                if nv.path.is_ident("module") {
                    if let syn::Expr::Lit(lit) = &nv.value {
                        if let Lit::Str(s) = &lit.lit {
                            return Ok(s.value());
                        }
                    }
                }
            }
        }
    }
    // Default module path
    Ok("ros_z_msgs_py.types".to_string())
}

/// Check if a type is u8
fn is_u8_type(ty: &Type) -> bool {
    if let Type::Path(type_path) = ty {
        if let Some(last_segment) = type_path.path.segments.last() {
            return last_segment.ident == "u8";
        }
    }
    false
}

/// Convert Rust field name to Python attribute name
/// Handles r#type -> type conversion for Rust keywords
fn field_name_to_py_attr(ident: &Ident) -> String {
    let name = ident.to_string();
    // Strip r# prefix used for Rust keywords
    if let Some(stripped) = name.strip_prefix("r#") {
        stripped.to_string()
    } else {
        name
    }
}
