use std::ffi::{CStr, c_char};
use std::ptr;

use crate::ros::{RCL_RET_INVALID_ARGUMENT, RCL_RET_OK};

// Validation result constants
pub const RCL_TOPIC_NAME_VALID: i32 = 0;
pub const RCL_TOPIC_NAME_INVALID_IS_EMPTY_STRING: i32 = 1;
pub const RCL_TOPIC_NAME_INVALID_ENDS_WITH_FORWARD_SLASH: i32 = 2;
pub const RCL_TOPIC_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS: i32 = 3;
pub const RCL_TOPIC_NAME_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER: i32 = 4;
pub const RCL_TOPIC_NAME_INVALID_UNMATCHED_CURLY_BRACE: i32 = 5;
pub const RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE: i32 = 6;
pub const RCL_TOPIC_NAME_INVALID_TILDE_NOT_FOLLOWED_BY_FORWARD_SLASH: i32 = 7;
pub const RCL_TOPIC_NAME_INVALID_SUBSTITUTION_CONTAINS_UNALLOWED_CHARACTERS: i32 = 8;
pub const RCL_TOPIC_NAME_INVALID_SUBSTITUTION_STARTS_WITH_NUMBER: i32 = 9;

/// Check if a character is allowed in a topic name token
fn is_valid_topic_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

/// Validate a topic name
///
/// Returns (validation_result, invalid_index_option)
fn validate_topic_name_impl(topic: &str) -> (i32, Option<usize>) {
    // Check for empty string
    if topic.is_empty() {
        return (RCL_TOPIC_NAME_INVALID_IS_EMPTY_STRING, Some(0));
    }

    // Check for trailing slash
    if topic.ends_with('/') {
        return (
            RCL_TOPIC_NAME_INVALID_ENDS_WITH_FORWARD_SLASH,
            Some(topic.len() - 1),
        );
    }

    let chars: Vec<char> = topic.chars().collect();
    let mut i = 0;
    let mut in_substitution = false;
    let mut substitution_start = 0;
    let mut token_start = 0;

    while i < chars.len() {
        let c = chars[i];

        // Handle curly braces for substitutions
        if c == '{' {
            if in_substitution {
                // Nested braces not allowed
                return (
                    RCL_TOPIC_NAME_INVALID_SUBSTITUTION_CONTAINS_UNALLOWED_CHARACTERS,
                    Some(i),
                );
            }
            in_substitution = true;
            substitution_start = i + 1;
            i += 1;
            continue;
        }

        if c == '}' {
            if !in_substitution {
                // Unmatched closing brace
                return (RCL_TOPIC_NAME_INVALID_UNMATCHED_CURLY_BRACE, Some(i));
            }

            // Validate substitution content
            let subst_content: String = chars[substitution_start..i].iter().collect();
            if subst_content.is_empty() {
                return (
                    RCL_TOPIC_NAME_INVALID_SUBSTITUTION_CONTAINS_UNALLOWED_CHARACTERS,
                    Some(substitution_start),
                );
            }

            // Check if substitution starts with number
            if subst_content.chars().next().unwrap().is_ascii_digit() {
                return (
                    RCL_TOPIC_NAME_INVALID_SUBSTITUTION_STARTS_WITH_NUMBER,
                    Some(substitution_start),
                );
            }

            // Check for invalid characters in substitution
            for (idx, sc) in subst_content.chars().enumerate() {
                if !is_valid_topic_char(sc) {
                    return (
                        RCL_TOPIC_NAME_INVALID_SUBSTITUTION_CONTAINS_UNALLOWED_CHARACTERS,
                        Some(substitution_start + idx),
                    );
                }
            }

            in_substitution = false;
            i += 1;
            continue;
        }

        if in_substitution {
            // Inside substitution, handled above
            i += 1;
            continue;
        }

        // Handle tilde
        if c == '~' {
            // Tilde is only valid at position 0
            if i == 0 {
                // Must be followed by '/' or be the entire topic
                if chars.len() > 1 && chars[i + 1] != '/' {
                    return (
                        RCL_TOPIC_NAME_INVALID_TILDE_NOT_FOLLOWED_BY_FORWARD_SLASH,
                        Some(i + 1),
                    );
                }
            } else {
                // Tilde anywhere else is misplaced
                return (RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE, Some(i));
            }
        } else if c == '/' {
            token_start = i + 1;
        } else {
            // Regular character
            // Check for invalid characters
            if !is_valid_topic_char(c) && c != '{' && c != '}' {
                return (
                    RCL_TOPIC_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS,
                    Some(i),
                );
            }

            // Check if token starts with a number
            if i == token_start && c.is_ascii_digit() {
                return (
                    RCL_TOPIC_NAME_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER,
                    Some(i),
                );
            }
        }

        i += 1;
    }

    // Check for unmatched opening brace
    if in_substitution {
        return (
            RCL_TOPIC_NAME_INVALID_UNMATCHED_CURLY_BRACE,
            Some(substitution_start - 1),
        );
    }

    (RCL_TOPIC_NAME_VALID, None)
}

/// Validate a topic name from C
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_validate_topic_name(
    topic_name: *const c_char,
    validation_result: *mut i32,
    invalid_index: *mut usize,
) -> i32 {
    // Check arguments
    if topic_name.is_null() || validation_result.is_null() {
        return RCL_RET_INVALID_ARGUMENT as i32;
    }

    // Convert C string to Rust string
    let topic_str = match unsafe { CStr::from_ptr(topic_name) }.to_str() {
        Ok(s) => s,
        Err(_) => {
            unsafe { *validation_result = RCL_TOPIC_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS };
            if !invalid_index.is_null() {
                unsafe { *invalid_index = 0 };
            }
            return RCL_RET_OK as i32;
        }
    };

    let (result, idx) = validate_topic_name_impl(topic_str);

    unsafe { *validation_result = result };

    if let Some(index) = idx {
        if !invalid_index.is_null() {
            unsafe { *invalid_index = index };
        }
    }

    RCL_RET_OK as i32
}

/// Get a string description of a validation result
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_topic_name_validation_result_string(
    validation_result: i32,
) -> *const c_char {
    match validation_result {
        RCL_TOPIC_NAME_VALID => ptr::null(),
        RCL_TOPIC_NAME_INVALID_IS_EMPTY_STRING => {
            c"topic name is an empty string".as_ptr()
        }
        RCL_TOPIC_NAME_INVALID_ENDS_WITH_FORWARD_SLASH => {
            c"topic name ends with a forward slash".as_ptr()
        }
        RCL_TOPIC_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS => {
            c"topic name contains unallowed characters".as_ptr()
        }
        RCL_TOPIC_NAME_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER => {
            c"topic name token starts with a number".as_ptr()
        }
        RCL_TOPIC_NAME_INVALID_UNMATCHED_CURLY_BRACE => {
            c"topic name has an unmatched curly brace".as_ptr()
        }
        RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE => {
            c"topic name has a misplaced tilde".as_ptr()
        }
        RCL_TOPIC_NAME_INVALID_TILDE_NOT_FOLLOWED_BY_FORWARD_SLASH => {
            c"tilde in topic name is not followed by a forward slash".as_ptr()
        }
        RCL_TOPIC_NAME_INVALID_SUBSTITUTION_CONTAINS_UNALLOWED_CHARACTERS => {
            c"topic name substitution contains unallowed characters".as_ptr()
        }
        RCL_TOPIC_NAME_INVALID_SUBSTITUTION_STARTS_WITH_NUMBER => {
            c"topic name substitution starts with a number".as_ptr()
        }
        _ => c"unknown result code for rcl topic name validation".as_ptr(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_topics() {
        let valid_topics = vec![
            "foo",
            "abc123",
            "_foo",
            "Foo",
            "BAR",
            "~",
            "foo/bar",
            "~/foo",
            "{foo}_bar",
            "foo/{ping}/bar",
            "foo/_bar",
            "foo_/bar",
            "foo_",
            "/foo",
            "/bar/baz",
            "/_private/thing",
            "/public_namespace/_private/thing",
            "{foo1}",
            "{foo_bar}",
            "{_bar}",
        ];

        for topic in valid_topics {
            let (result, idx) = validate_topic_name_impl(topic);
            assert_eq!(
                result, RCL_TOPIC_NAME_VALID,
                "Topic '{}' should be valid but got error at index {:?}",
                topic, idx
            );
        }
    }

    #[test]
    fn test_invalid_topics() {
        let invalid_topics = vec![
            ("", RCL_TOPIC_NAME_INVALID_IS_EMPTY_STRING, 0),
            ("123abc", RCL_TOPIC_NAME_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER, 0),
            (" ", RCL_TOPIC_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS, 0),
            ("foo bar", RCL_TOPIC_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS, 3),
            ("/~", RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE, 1),
            ("~foo", RCL_TOPIC_NAME_INVALID_TILDE_NOT_FOLLOWED_BY_FORWARD_SLASH, 1),
            ("foo~", RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE, 3),
            ("foo/", RCL_TOPIC_NAME_INVALID_ENDS_WITH_FORWARD_SLASH, 3),
        ];

        for (topic, expected_result, expected_index) in invalid_topics {
            let (result, idx) = validate_topic_name_impl(topic);
            assert_eq!(
                result, expected_result,
                "Topic '{}' should fail with {} but got {}",
                topic, expected_result, result
            );
            assert_eq!(
                idx,
                Some(expected_index),
                "Topic '{}' should have invalid index {}",
                topic,
                expected_index
            );
        }
    }
}
