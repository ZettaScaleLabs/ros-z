package rosz

/*
#include <stdlib.h>
#include "ros_z_ffi.h"

extern ros_z_ServiceCallback getServiceCallback();
*/
import "C"
import (
	"fmt"
	"runtime"
	"runtime/cgo"
	"sync"
	"time"
	"unsafe"
)

// Service represents a ROS 2 service (request/response pattern)
type Service interface {
	Message
	// GetRequest returns the request message type
	GetRequest() Message
	// GetResponse returns the response message type
	GetResponse() Message
}

// ServiceClient calls ROS 2 services
type ServiceClient struct {
	handle    *C.ros_z_service_client_t
	node      *Node
	service   string
	closeOnce sync.Once
}

// ServiceServer responds to ROS 2 service requests
type ServiceServer struct {
	handle    *C.ros_z_service_server_t
	node      *Node
	service   string
	closure   *serviceClosure
	closeOnce sync.Once
}

// ServiceClientBuilder builds a ServiceClient
type ServiceClientBuilder struct {
	node    *Node
	service string
}

// ServiceServerBuilder builds a ServiceServer
type ServiceServerBuilder struct {
	node    *Node
	service string
}

// CreateServiceClient creates a new service client builder
func (n *Node) CreateServiceClient(service string) *ServiceClientBuilder {
	return &ServiceClientBuilder{
		node:    n,
		service: service,
	}
}

// CreateServiceServer creates a new service server builder
func (n *Node) CreateServiceServer(service string) *ServiceServerBuilder {
	return &ServiceServerBuilder{
		node:    n,
		service: service,
	}
}

// Build creates the service client
func (b *ServiceClientBuilder) Build(svc Service) (*ServiceClient, error) {
	serviceC := C.CString(b.service)
	defer C.free(unsafe.Pointer(serviceC))

	// Use the service-level TypeName/TypeHash (DDS format) so the key expression
	// matches rmw_zenoh_cpp and the Rust ros-z native API.
	svcTypeC := C.CString(svc.TypeName())
	defer C.free(unsafe.Pointer(svcTypeC))

	svcHashC := C.CString(svc.TypeHash())
	defer C.free(unsafe.Pointer(svcHashC))

	handle := C.ros_z_service_client_create(
		b.node.handle,
		serviceC,
		svcTypeC, svcHashC,
		svcTypeC, svcHashC,
	)

	if handle == nil {
		return nil, fmt.Errorf("%w: service client for %s", ErrBuildFailed, b.service)
	}

	client := &ServiceClient{
		handle:  handle,
		node:    b.node,
		service: b.service,
	}
	runtime.SetFinalizer(client, (*ServiceClient).Close)

	return client, nil
}

// CallRaw makes a synchronous service call with raw CDR bytes.
// Returns the raw CDR response bytes.
func (c *ServiceClient) CallRaw(requestBytes []byte, timeoutMs uint64) ([]byte, error) {
	if c.handle == nil {
		return nil, fmt.Errorf("service client is closed")
	}

	if len(requestBytes) == 0 {
		return nil, fmt.Errorf("empty request")
	}

	// Pin the request data to prevent GC relocation during the C call
	pinner := &runtime.Pinner{}
	defer pinner.Unpin()
	pinner.Pin(&requestBytes[0])

	var respPtr *C.uint8_t
	var respLen C.uintptr_t

	result := C.ros_z_service_client_call(
		c.handle,
		(*C.uint8_t)(unsafe.Pointer(&requestBytes[0])),
		C.uintptr_t(len(requestBytes)),
		&respPtr,
		&respLen,
		C.uint64_t(timeoutMs),
	)

	logger.Debug("ros_z_service_client_call",
		"service", c.service, "req_len", len(requestBytes),
		"resp_len", int(respLen), "rc", int(result))

	if result != 0 {
		code := ErrorCode(result)
		if code == ErrorCodeServiceTimeout {
			return nil, NewRoszError(ErrorCodeServiceTimeout,
				fmt.Sprintf("service[%s] call timed out", c.service))
		}
		return nil, NewRoszError(ErrorCodeServiceCallFailed,
			fmt.Sprintf("service[%s] call failed (rc=%d)", c.service, result))
	}

	respBytes := C.GoBytes(unsafe.Pointer(respPtr), C.int(respLen))
	C.ros_z_free_bytes((*C.uint8_t)(respPtr), C.uintptr_t(respLen))

	return respBytes, nil
}

// Call makes a synchronous service call with a default 5 second timeout.
// The request is serialized via CDR, sent, and the raw response bytes are returned.
func (c *ServiceClient) Call(request Message) ([]byte, error) {
	reqBytes, err := request.SerializeCDR()
	if err != nil {
		return nil, fmt.Errorf("failed to serialize request: %w", err)
	}

	return c.CallRaw(reqBytes, 5000)
}

// CallWithTimeout makes a synchronous service call with a custom timeout.
// The request is serialized via CDR, sent, and the raw response bytes are returned.
func (c *ServiceClient) CallWithTimeout(request Message, timeout time.Duration) ([]byte, error) {
	reqBytes, err := request.SerializeCDR()
	if err != nil {
		return nil, fmt.Errorf("failed to serialize request: %w", err)
	}

	timeoutMs := uint64(timeout.Milliseconds())
	if timeoutMs == 0 {
		timeoutMs = 1
	}
	return c.CallRaw(reqBytes, timeoutMs)
}

// Close destroys the service client
func (c *ServiceClient) Close() error {
	var err error
	c.closeOnce.Do(func() {
		if c.handle == nil {
			return
		}
		result := C.ros_z_service_client_destroy(c.handle)
		c.handle = nil
		if result != 0 {
			err = fmt.Errorf("service client close failed with code %d", result)
		}
	})
	return err
}

// serviceClosure wraps a service callback with pinning for safe C access
type serviceClosure struct {
	name     string // service name, for logging
	callback func([]byte) ([]byte, error)
	handle   cgo.Handle
	pinner   *runtime.Pinner
}

// newServiceClosure creates a pinned service closure
func newServiceClosure(name string, callback func([]byte) ([]byte, error)) *serviceClosure {
	sc := &serviceClosure{
		name:     name,
		callback: callback,
		handle:   cgo.NewHandle(callback),
	}
	sc.pinner = &runtime.Pinner{}
	sc.pinner.Pin(sc)
	return sc
}

// drop cleans up the service closure
func (sc *serviceClosure) drop() {
	sc.handle.Delete()
	sc.pinner.Unpin()
}

// Build creates the service server with callback.
// The callback receives raw request bytes and must return raw response bytes.
// Note: the callback is invoked on a C/Rust thread — avoid long blocking operations.
func (b *ServiceServerBuilder) Build(svc Service, callback func([]byte) ([]byte, error)) (*ServiceServer, error) {
	serviceC := C.CString(b.service)
	defer C.free(unsafe.Pointer(serviceC))

	// Use the service-level TypeName/TypeHash (DDS format) so the key expression
	// matches rmw_zenoh_cpp and the Rust ros-z native API.
	svcTypeC := C.CString(svc.TypeName())
	defer C.free(unsafe.Pointer(svcTypeC))

	svcHashC := C.CString(svc.TypeHash())
	defer C.free(unsafe.Pointer(svcHashC))

	// Create pinned closure
	closure := newServiceClosure(b.service, callback)

	handle := C.ros_z_service_server_create(
		b.node.handle,
		serviceC,
		svcTypeC, svcHashC,
		svcTypeC, svcHashC,
		C.getServiceCallback(),
		C.uintptr_t(uintptr(unsafe.Pointer(closure))),
	)

	if handle == nil {
		closure.drop()
		return nil, fmt.Errorf("%w: service server for %s", ErrBuildFailed, b.service)
	}

	server := &ServiceServer{
		handle:  handle,
		node:    b.node,
		service: b.service,
		closure: closure,
	}
	runtime.SetFinalizer(server, (*ServiceServer).Close)

	return server, nil
}

// Close destroys the service server
func (s *ServiceServer) Close() error {
	var err error
	s.closeOnce.Do(func() {
		if s.handle == nil {
			return
		}
		result := C.ros_z_service_server_destroy(s.handle)
		s.handle = nil
		if s.closure != nil {
			s.closure.drop()
			s.closure = nil
		}
		if result != 0 {
			err = fmt.Errorf("service server close failed with code %d", result)
		}
	})
	return err
}

//export goServiceCallback
func goServiceCallback(userData C.uintptr_t, reqData *C.uint8_t, reqLen C.size_t, respData **C.uint8_t, respLen *C.size_t) (rc C.int32_t) {
	// Cast userData back to serviceClosure pointer
	closure := (*serviceClosure)(unsafe.Pointer(uintptr(userData)))

	// Copy request data to Go before entering safeCall.
	goReqData := C.GoBytes(unsafe.Pointer(reqData), C.int(reqLen))

	logger.Debug("goServiceCallback", "service", closure.name, "req_len", int(reqLen))

	err := safeCall(func() error {
		// Call user callback
		goRespData, err := closure.callback(goReqData)
		if err != nil {
			logger.Error("service callback error", "service", closure.name, "err", err)
			return err
		}

		if len(goRespData) == 0 {
			logger.Error("service callback returned empty response", "service", closure.name)
			return fmt.Errorf("empty response")
		}

		// Allocate response in C memory (will be freed by Rust)
		*respLen = C.size_t(len(goRespData))
		*respData = (*C.uint8_t)(C.CBytes(goRespData))
		return nil
	})

	if err != nil {
		return -1
	}
	return 0
}
