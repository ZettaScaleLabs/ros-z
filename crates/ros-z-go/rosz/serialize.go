package rosz

/*
#include <stdlib.h>
#include "ros_z_ffi.h"
*/
import "C"
import (
	"fmt"
	"unsafe"
)

// SerializeMessage serializes raw bytes to CDR using Rust
func SerializeMessage(typeName string, raw []byte) ([]byte, error) {
	if len(raw) == 0 {
		return nil, fmt.Errorf("cannot serialize empty input for %s", typeName)
	}

	typeNameC := C.CString(typeName)
	defer C.free(unsafe.Pointer(typeNameC))

	var outPtr *C.uint8_t
	var outLen C.size_t

	result := C.ros_z_serialize(
		typeNameC,
		(*C.uint8_t)(unsafe.Pointer(&raw[0])),
		C.size_t(len(raw)),
		&outPtr,
		&outLen,
	)

	if result != 0 {
		return nil, fmt.Errorf("serialization failed with code %d", result)
	}

	// Copy to Go slice and free C memory
	goBytes := C.GoBytes(unsafe.Pointer(outPtr), C.int(outLen))
	C.ros_z_free_bytes(outPtr, outLen)

	return goBytes, nil
}

// DeserializeMessage deserializes CDR bytes to raw format using Rust
func DeserializeMessage(typeName string, cdr []byte) ([]byte, error) {
	if len(cdr) == 0 {
		return nil, fmt.Errorf("cannot deserialize empty CDR data for %s", typeName)
	}

	typeNameC := C.CString(typeName)
	defer C.free(unsafe.Pointer(typeNameC))

	var outPtr *C.uint8_t
	var outLen C.size_t

	result := C.ros_z_deserialize(
		typeNameC,
		(*C.uint8_t)(unsafe.Pointer(&cdr[0])),
		C.size_t(len(cdr)),
		&outPtr,
		&outLen,
	)

	if result != 0 {
		return nil, fmt.Errorf("deserialization failed with code %d", result)
	}

	goBytes := C.GoBytes(unsafe.Pointer(outPtr), C.int(outLen))
	C.ros_z_free_bytes(outPtr, outLen)

	return goBytes, nil
}
