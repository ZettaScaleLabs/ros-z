package rosz

import (
	"sync"
	"testing"
)

// TestNodeOwnedSubsField verifies that Node has the ownedSubs and subsMu fields
// required for callback subscriber lifetime management.
//
// Full integration test (BuildWithCallback without assigning result) requires a
// live Zenoh session and is covered by CI integration tests.
func TestNodeOwnedSubsField(t *testing.T) {
	// Verify the Node struct has the correct fields by constructing one directly.
	// This will fail to compile if ownedSubs or subsMu are removed.
	node := &Node{
		ownedSubs: nil,
		subsMu:    sync.Mutex{},
	}

	if node.ownedSubs != nil {
		t.Error("ownedSubs should be nil initially")
	}

	// Simulate what BuildWithCallback does: store a subscriber in the node.
	// In production, this is a *Subscriber; here we use interface{} directly.
	mockSub := &struct{ closed bool }{}

	node.subsMu.Lock()
	node.ownedSubs = append(node.ownedSubs, mockSub)
	node.subsMu.Unlock()

	if len(node.ownedSubs) != 1 {
		t.Errorf("Expected 1 owned sub, got %d", len(node.ownedSubs))
	}

	t.Log("Node.ownedSubs ownership pattern verified")
}

// TestCallbackSubscriberLifetimePattern documents the expected behavior:
// a subscriber created with BuildWithCallback must remain active even
// when the caller does not store the returned *Subscriber.
//
// The node keeps the subscription alive via ownedSubs until Node.Close().
// This matches rmw_zenoh_cpp's NodeData::subs_ pattern.
func TestCallbackSubscriberLifetimePattern(t *testing.T) {
	// Demonstrate the pattern using the mock closure infrastructure.
	var callCount int
	handler := func(data []byte) {
		callCount++
	}

	// Simulate the BuildWithCallback closure creation (no real node needed).
	closure := newClosure(handler, nil)
	if closure == nil {
		t.Fatal("closure should not be nil")
	}

	// Simulate message delivery.
	closure.call([]byte("hello"))
	if callCount != 1 {
		t.Errorf("Expected handler called once, got %d", callCount)
	}

	// Cleanup closure (normally done by Rust on subscriber destroy).
	closure.drop()

	t.Log("Callback closure lifetime pattern verified")
}
