package rosz

/*
#include <stdlib.h>
#include "ros_z_ffi.h"
*/
import "C"
import (
	"fmt"
	"runtime"
	"sync"
	"unsafe"
)

// Node represents a ROS 2 node
type Node struct {
	handle    *C.ros_z_node_t
	ctx       *Context
	closeOnce sync.Once
}

// NodeBuilder builds a Node
type NodeBuilder struct {
	ctx                          *Context
	name                         string
	namespace                    string
	enableTypeDescriptionService bool
}

// WithNamespace sets the node namespace
func (b *NodeBuilder) WithNamespace(ns string) *NodeBuilder {
	b.namespace = ns
	return b
}

// WithTypeDescriptionService enables the type description service for this node.
// When enabled, `ros2 topic info --verbose` can query type descriptions from this node.
func (b *NodeBuilder) WithTypeDescriptionService() *NodeBuilder {
	b.enableTypeDescriptionService = true
	return b
}

// Build creates the node
func (b *NodeBuilder) Build() (*Node, error) {
	var handle *C.ros_z_node_t

	if b.enableTypeDescriptionService {
		// Use config-based creation
		cName := C.CString(b.name)
		defer C.free(unsafe.Pointer(cName))

		var cfg C.ros_z_node_config_t
		cfg.name = cName
		cfg.enable_type_description_service = C.bool(b.enableTypeDescriptionService)

		if b.namespace != "" {
			cNamespace := C.CString(b.namespace)
			defer C.free(unsafe.Pointer(cNamespace))
			cfg.namespace_ = cNamespace
		}

		handle = C.ros_z_node_create_with_config(b.ctx.handle, &cfg)
	} else {
		// Simple path
		cName := C.CString(b.name)
		defer C.free(unsafe.Pointer(cName))

		var cNamespace *C.char
		if b.namespace != "" {
			cNamespace = C.CString(b.namespace)
			defer C.free(unsafe.Pointer(cNamespace))
		}

		handle = C.ros_z_node_create(b.ctx.handle, cName, cNamespace)
	}

	if handle == nil {
		return nil, fmt.Errorf("%w: node %s", ErrBuildFailed, b.name)
	}

	node := &Node{
		handle: handle,
		ctx:    b.ctx,
	}
	runtime.SetFinalizer(node, (*Node).Close)

	return node, nil
}

// CreatePublisher creates a new publisher builder
func (n *Node) CreatePublisher(topic string) *PublisherBuilder {
	return &PublisherBuilder{
		node:  n,
		topic: topic,
	}
}

// CreateSubscriber creates a new subscriber builder
func (n *Node) CreateSubscriber(topic string) *SubscriberBuilder {
	return &SubscriberBuilder{
		node:  n,
		topic: topic,
	}
}

// Close destroys the node
func (n *Node) Close() error {
	var err error
	n.closeOnce.Do(func() {
		if n.handle == nil {
			return
		}
		result := C.ros_z_node_destroy(n.handle)
		n.handle = nil
		if result != 0 {
			err = fmt.Errorf("node close failed with code %d", result)
		}
	})
	return err
}
