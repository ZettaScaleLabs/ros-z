package rosz

import (
	"fmt"
)

// ErrorCode represents FFI error codes returned from the Rust layer
type ErrorCode int32

const (
	// ErrorCodeSuccess indicates the operation completed successfully
	ErrorCodeSuccess ErrorCode = 0

	// ErrorCodeNullPointer indicates a null pointer was passed to FFI
	ErrorCodeNullPointer ErrorCode = -1

	// ErrorCodeInvalidUtf8 indicates invalid UTF-8 in string conversion
	ErrorCodeInvalidUtf8 ErrorCode = -2

	// ErrorCodeSessionClosed indicates the Zenoh session is closed
	ErrorCodeSessionClosed ErrorCode = -3

	// ErrorCodePublishFailed indicates message publishing failed
	ErrorCodePublishFailed ErrorCode = -4

	// ErrorCodeSerializationFailed indicates CDR serialization failed
	ErrorCodeSerializationFailed ErrorCode = -5

	// ErrorCodeSubscribeFailed indicates subscriber creation failed
	ErrorCodeSubscribeFailed ErrorCode = -6

	// ErrorCodeNodeCreationFailed indicates node creation failed
	ErrorCodeNodeCreationFailed ErrorCode = -7

	// ErrorCodeContextCreationFailed indicates context creation failed
	ErrorCodeContextCreationFailed ErrorCode = -8

	// ErrorCodeDeserializationFailed indicates CDR deserialization failed
	ErrorCodeDeserializationFailed ErrorCode = -9

	// ErrorCodeBuildFailed indicates a builder failed to construct the entity (FFI returned null)
	ErrorCodeBuildFailed ErrorCode = -10

	// ErrorCodeUnknown indicates an unknown error occurred
	ErrorCodeUnknown ErrorCode = -100
)

// RoszError represents a structured error from the ros-z FFI layer
type RoszError struct {
	code ErrorCode
	msg  string
}

// Error implements the error interface
func (e RoszError) Error() string {
	return fmt.Sprintf("%s (code: %d)", e.msg, e.code)
}

// Code returns the FFI error code
func (e RoszError) Code() ErrorCode {
	return e.code
}

// Message returns the error message without the code
func (e RoszError) Message() string {
	return e.msg
}

// NewRoszError creates a new RoszError with the given code and message
func NewRoszError(code ErrorCode, msg string) RoszError {
	return RoszError{code: code, msg: msg}
}

// Is reports whether target matches this error by comparing error codes.
// This enables errors.Is() support for RoszError.
// Uses direct type assertion (not errors.As) to avoid recursive chain walking.
func (e RoszError) Is(target error) bool {
	t, ok := target.(RoszError)
	if ok {
		return e.code == t.code
	}
	return false
}

// Sentinel errors for common failure modes
var (
	// ErrBuildFailed is returned when a builder's Build() call fails (FFI returned null pointer).
	// Use errors.Is(err, rosz.ErrBuildFailed) to detect construction failures.
	ErrBuildFailed = NewRoszError(ErrorCodeBuildFailed, "failed to build entity")
)
