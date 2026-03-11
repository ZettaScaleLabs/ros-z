package rosz

import (
	"errors"
	"fmt"
	"testing"
)

func TestRoszError(t *testing.T) {
	err := NewRoszError(ErrorCodePublishFailed, "publish failed")

	if err.Code() != ErrorCodePublishFailed {
		t.Errorf("Code() = %d, want %d", err.Code(), ErrorCodePublishFailed)
	}

	if err.Message() != "publish failed" {
		t.Errorf("Message() = %q, want %q", err.Message(), "publish failed")
	}

	expected := "publish failed (code: -4)"
	if err.Error() != expected {
		t.Errorf("Error() = %q, want %q", err.Error(), expected)
	}
}

func TestRoszErrorIsError(t *testing.T) {
	// Verify RoszError implements error interface
	var err error = NewRoszError(ErrorCodePublishFailed, "test")
	if err.Error() == "" {
		t.Error("RoszError should implement error interface")
	}
}

func TestRoszErrorTypeAssertion(t *testing.T) {
	var err error = NewRoszError(ErrorCodeSubscribeFailed, "subscribe failed")

	roszErr, ok := err.(RoszError)
	if !ok {
		t.Fatal("type assertion to RoszError failed")
	}

	if roszErr.Code() != ErrorCodeSubscribeFailed {
		t.Errorf("Code() = %d, want %d", roszErr.Code(), ErrorCodeSubscribeFailed)
	}
}

func TestRoszErrorWithErrors(t *testing.T) {
	err := NewRoszError(ErrorCodeBuildFailed, "build failed")

	// errors.Is matches by error code (message is ignored)
	if !errors.Is(err, NewRoszError(ErrorCodeBuildFailed, "different message")) {
		t.Error("errors.Is should match RoszError with same code")
	}

	// errors.Is should not match different codes
	if errors.Is(err, NewRoszError(ErrorCodePublishFailed, "build failed")) {
		t.Error("errors.Is should not match RoszError with different code")
	}

	// Sentinel errors should work with errors.Is
	if !errors.Is(err, ErrBuildFailed) {
		t.Error("errors.Is should match sentinel ErrBuildFailed")
	}

	// errors.As should work
	var targetErr RoszError
	if !errors.As(err, &targetErr) {
		t.Error("errors.As should work for RoszError")
	}

	if targetErr.Code() != ErrorCodeBuildFailed {
		t.Errorf("Code() after errors.As = %d, want %d", targetErr.Code(), ErrorCodeBuildFailed)
	}
}

func TestRoszErrorIsNoRecursion(t *testing.T) {
	// Wrapping a RoszError should not cause infinite recursion in Is()
	inner := NewRoszError(ErrorCodeBuildFailed, "inner failure")
	wrapped := fmt.Errorf("outer: %w", inner)

	// errors.Is walks the chain and calls Is() — must not infinite-loop
	if !errors.Is(wrapped, ErrBuildFailed) {
		t.Error("errors.Is should find ErrBuildFailed through wrapped chain")
	}

	// Double-wrapped
	doubleWrapped := fmt.Errorf("double: %w", wrapped)
	if !errors.Is(doubleWrapped, ErrBuildFailed) {
		t.Error("errors.Is should find ErrBuildFailed through double-wrapped chain")
	}

	// Different code should not match
	if errors.Is(doubleWrapped, NewRoszError(ErrorCodePublishFailed, "")) {
		t.Error("errors.Is should not match different error code in chain")
	}
}

func TestErrorCodeConstants(t *testing.T) {
	// Verify error code values match Rust FFI
	tests := []struct {
		code     ErrorCode
		expected int32
	}{
		{ErrorCodeSuccess, 0},
		{ErrorCodeNullPointer, -1},
		{ErrorCodeInvalidUtf8, -2},
		{ErrorCodeSessionClosed, -3},
		{ErrorCodePublishFailed, -4},
		{ErrorCodeSerializationFailed, -5},
		{ErrorCodeSubscribeFailed, -6},
		{ErrorCodeNodeCreationFailed, -7},
		{ErrorCodeContextCreationFailed, -8},
		{ErrorCodeDeserializationFailed, -9},
		{ErrorCodeBuildFailed, -10},
		{ErrorCodeUnknown, -100},
	}

	for _, tt := range tests {
		if int32(tt.code) != tt.expected {
			t.Errorf("ErrorCode value mismatch: got %d, want %d", tt.code, tt.expected)
		}
	}
}
