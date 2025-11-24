#![cfg(feature = "test-core")]

use std::{ffi::CString, ptr};

use rcl_z::{ros::RCL_RET_OK, validate_topic_name::*};

#[test]
fn test_normal() {
    unsafe {
        // passing without invalid_index
        {
            let mut validation_result: i32 = 0;
            let topic = CString::new("topic").unwrap();
            let ret =
                rcl_validate_topic_name(topic.as_ptr(), &mut validation_result, ptr::null_mut());
            assert_eq!(RCL_RET_OK as i32, ret);
            assert_eq!(RCL_TOPIC_NAME_VALID, validation_result);
            assert_eq!(
                ptr::null(),
                rcl_topic_name_validation_result_string(validation_result)
            );
        }

        // passing with invalid_index
        {
            let mut validation_result: i32 = 0;
            let mut invalid_index: usize = 42;
            let topic = CString::new("topic").unwrap();
            let ret =
                rcl_validate_topic_name(topic.as_ptr(), &mut validation_result, &mut invalid_index);
            assert_eq!(RCL_RET_OK as i32, ret);
            assert_eq!(RCL_TOPIC_NAME_VALID, validation_result);
            assert_eq!(42, invalid_index); // ensure invalid_index is not assigned on success
            assert_eq!(
                ptr::null(),
                rcl_topic_name_validation_result_string(validation_result)
            );
        }

        // failing with invalid_index
        {
            let mut validation_result: i32 = 0;
            let mut invalid_index: usize = 42;
            let topic = CString::new("").unwrap();
            let ret =
                rcl_validate_topic_name(topic.as_ptr(), &mut validation_result, &mut invalid_index);
            assert_eq!(RCL_RET_OK as i32, ret);
            assert_eq!(RCL_TOPIC_NAME_INVALID_IS_EMPTY_STRING, validation_result);
            assert_eq!(0, invalid_index);
            assert_ne!(
                ptr::null(),
                rcl_topic_name_validation_result_string(validation_result)
            );
        }
    }
}

#[test]
fn test_invalid_arguments() {
    unsafe {
        // pass null for topic string
        {
            let mut validation_result: i32 = 0;
            let ret = rcl_validate_topic_name(ptr::null(), &mut validation_result, ptr::null_mut());
            assert_eq!(rcl_z::ros::RCL_RET_INVALID_ARGUMENT as i32, ret);
        }

        // pass null for validation_result
        {
            let topic = CString::new("topic").unwrap();
            let ret = rcl_validate_topic_name(topic.as_ptr(), ptr::null_mut(), ptr::null_mut());
            assert_eq!(rcl_z::ros::RCL_RET_INVALID_ARGUMENT as i32, ret);
        }
    }
}

#[test]
fn test_various_valid_topics() {
    let topics_that_should_pass = vec![
        // examples from the design doc:
        //   http://design.ros2.org/articles/topic_and_service_names.html#ros-2-name-examples
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
        // these are further corner cases identified:
        "{foo1}",
        "{foo_bar}",
        "{_bar}",
    ];

    unsafe {
        for topic in topics_that_should_pass {
            let mut validation_result: i32 = 0;
            let mut invalid_index: usize = 42;
            let topic_cstr = CString::new(topic).unwrap();
            let ret = rcl_validate_topic_name(
                topic_cstr.as_ptr(),
                &mut validation_result,
                &mut invalid_index,
            );
            assert_eq!(RCL_RET_OK as i32, ret);
            assert_eq!(
                RCL_TOPIC_NAME_VALID, validation_result,
                "'{}' should have passed but got validation result: {}",
                topic, validation_result
            );
            assert_eq!(
                42, invalid_index,
                "'{}' should not modify invalid_index on success",
                topic
            );
            let result_str = rcl_topic_name_validation_result_string(validation_result);
            assert_eq!(
                ptr::null(),
                result_str,
                "'{}' should return null for valid topic",
                topic
            );
        }

        // Test unknown validation result code
        let not_valid_validation_result = 5600;
        let result_str = rcl_topic_name_validation_result_string(not_valid_validation_result);
        assert_ne!(ptr::null(), result_str);
        let c_str = std::ffi::CStr::from_ptr(result_str);
        assert_eq!(
            "unknown result code for rcl topic name validation",
            c_str.to_str().unwrap()
        );
    }
}

#[test]
fn test_various_invalid_topics() {
    struct TopicCase {
        topic: &'static str,
        expected_validation_result: i32,
        expected_invalid_index: usize,
    }

    let topic_cases_that_should_fail = vec![
        // examples from the design doc:
        //   http://design.ros2.org/articles/topic_and_service_names.html#ros-2-name-examples
        TopicCase {
            topic: "123abc",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER,
            expected_invalid_index: 0,
        },
        TopicCase {
            topic: "123",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER,
            expected_invalid_index: 0,
        },
        TopicCase {
            topic: " ",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS,
            expected_invalid_index: 0,
        },
        TopicCase {
            topic: "foo bar",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS,
            expected_invalid_index: 3,
        },
        TopicCase {
            topic: "/~",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE,
            expected_invalid_index: 1,
        },
        TopicCase {
            topic: "~foo",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_TILDE_NOT_FOLLOWED_BY_FORWARD_SLASH,
            expected_invalid_index: 1,
        },
        TopicCase {
            topic: "foo~",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE,
            expected_invalid_index: 3,
        },
        TopicCase {
            topic: "foo~/bar",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE,
            expected_invalid_index: 3,
        },
        TopicCase {
            topic: "foo/~bar",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE,
            expected_invalid_index: 4,
        },
        TopicCase {
            topic: "foo/~/bar",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_MISPLACED_TILDE,
            expected_invalid_index: 4,
        },
        TopicCase {
            topic: "foo/",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_ENDS_WITH_FORWARD_SLASH,
            expected_invalid_index: 3,
        },
        // these are further corner cases identified:
        TopicCase {
            topic: "",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_IS_EMPTY_STRING,
            expected_invalid_index: 0,
        },
        TopicCase {
            topic: "foo/123bar",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER,
            expected_invalid_index: 4,
        },
        TopicCase {
            topic: "foo/bar}/baz",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_UNMATCHED_CURLY_BRACE,
            expected_invalid_index: 7,
        },
        TopicCase {
            topic: "foo/{bar",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_UNMATCHED_CURLY_BRACE,
            expected_invalid_index: 4,
        },
        TopicCase {
            topic: "{$}",
            expected_validation_result:
                RCL_TOPIC_NAME_INVALID_SUBSTITUTION_CONTAINS_UNALLOWED_CHARACTERS,
            expected_invalid_index: 1,
        },
        TopicCase {
            topic: "{{bar}_baz}",
            expected_validation_result:
                RCL_TOPIC_NAME_INVALID_SUBSTITUTION_CONTAINS_UNALLOWED_CHARACTERS,
            expected_invalid_index: 1,
        },
        TopicCase {
            topic: "foo/{bar/baz}",
            expected_validation_result:
                RCL_TOPIC_NAME_INVALID_SUBSTITUTION_CONTAINS_UNALLOWED_CHARACTERS,
            expected_invalid_index: 8,
        },
        TopicCase {
            topic: "{1foo}",
            expected_validation_result: RCL_TOPIC_NAME_INVALID_SUBSTITUTION_STARTS_WITH_NUMBER,
            expected_invalid_index: 1,
        },
    ];

    unsafe {
        for case in topic_cases_that_should_fail {
            let mut validation_result: i32 = 0;
            let mut invalid_index: usize = 0;
            let topic_cstr = CString::new(case.topic).unwrap();
            let ret = rcl_validate_topic_name(
                topic_cstr.as_ptr(),
                &mut validation_result,
                &mut invalid_index,
            );
            assert_eq!(RCL_RET_OK as i32, ret);
            assert_eq!(
                case.expected_validation_result, validation_result,
                "'{}' should have failed with '{}' but got '{}'",
                case.topic, case.expected_validation_result, validation_result
            );
            assert_eq!(
                case.expected_invalid_index, invalid_index,
                "Topic '{}' failed with '{}' at wrong index",
                case.topic, validation_result
            );
            let result_str = rcl_topic_name_validation_result_string(validation_result);
            assert_ne!(
                ptr::null(),
                result_str,
                "Topic '{}' should return non-null error string",
                case.topic
            );
        }
    }
}
