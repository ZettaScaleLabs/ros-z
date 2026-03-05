// crates/ros-z-go/examples/subscriber_channel/main.go
//
// This example demonstrates three delivery patterns for the same "chatter" topic:
//   - FifoChannel: buffered with backpressure (blocks when full)
//   - RingChannel: non-blocking, drops oldest when full
//   - Direct callback: zero allocation, lowest latency
//
// All three patterns subscribe to "chatter", so this example works with both
// the publisher example and a ROS 2 demo_nodes_cpp talker out of the box.
//
// Prerequisites:
// 1. Run `just codegen-bundled` to generate the message types
// 2. Build the Rust library with `just build-rust`
//
// Run this example with:
//
//	CGO_LDFLAGS="-L../../../target/release" go run main.go
package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/ZettaScaleLabs/ros-z/crates/ros-z-go/generated/std_msgs"
	"github.com/ZettaScaleLabs/ros-z/crates/ros-z-go/rosz"
)

func main() {
	log.Println("Starting ros-z Go channel-based subscriber example...")
	log.Println()
	log.Println("All three patterns subscribe to 'chatter':")
	log.Println("  1. FifoChannel  - Buffered with backpressure (blocks when full)")
	log.Println("  2. RingChannel  - Non-blocking (drops oldest when full)")
	log.Println("  3. Direct callback - Zero allocation (lowest latency)")
	log.Println()

	// Create a ROS 2 context
	ctx, err := rosz.NewContext().
		WithDomainID(0).
		Build()
	if err != nil {
		log.Fatalf("Failed to create context: %v", err)
	}
	defer ctx.Close()

	// Create a node
	node, err := ctx.CreateNode("go_listener_channel").Build()
	if err != nil {
		log.Fatalf("Failed to create node: %v", err)
	}
	defer node.Close()

	// Pattern 1: FifoChannel — buffered delivery with backpressure
	log.Println("Creating FIFO channel subscriber (buffer=10)...")
	fifoHandler := rosz.NewFifoChannel[[]byte](10)
	callback, drop, fifoCh := fifoHandler.ToCbDropHandler()

	fifoSub, err := node.CreateSubscriber("chatter").
		BuildWithCallback(&std_msgs.String{}, callback)
	if err != nil {
		log.Fatalf("Failed to create FIFO subscriber: %v", err)
	}
	defer func() {
		fifoSub.Close()
		drop()
	}()

	go func() {
		for data := range fifoCh {
			var msg std_msgs.String
			if err := msg.DeserializeCDR(data); err != nil {
				log.Printf("[FIFO] Deserialize error: %v", err)
				continue
			}
			log.Printf("[FIFO] Received: %s", msg.Data)
			// Simulate slow processing to show backpressure behaviour
			time.Sleep(200 * time.Millisecond)
		}
	}()

	// Pattern 2: RingChannel — non-blocking, drops oldest when full
	log.Println("Creating Ring channel subscriber (capacity=3)...")
	ringHandler := rosz.NewRingChannel[[]byte](3)
	ringCallback, ringDrop, ringCh := ringHandler.ToCbDropHandler()

	ringSub, err := node.CreateSubscriber("chatter").
		BuildWithCallback(&std_msgs.String{}, ringCallback)
	if err != nil {
		log.Fatalf("Failed to create Ring subscriber: %v", err)
	}
	defer func() {
		ringSub.Close()
		ringDrop()
	}()

	go func() {
		for data := range ringCh {
			var msg std_msgs.String
			if err := msg.DeserializeCDR(data); err != nil {
				log.Printf("[RING] Deserialize error: %v", err)
				continue
			}
			log.Printf("[RING] Received: %s (may have dropped older messages)", msg.Data)
		}
	}()

	// Pattern 3: Direct callback — lowest latency, no buffering
	log.Println("Creating direct callback subscriber...")
	directSub, err := node.CreateSubscriber("chatter").
		BuildWithCallback(&std_msgs.String{}, func(data []byte) {
			var msg std_msgs.String
			if err := msg.DeserializeCDR(data); err != nil {
				log.Printf("[DIRECT] Deserialize error: %v", err)
				return
			}
			log.Printf("[DIRECT] Received: %s", msg.Data)
		})
	if err != nil {
		log.Fatalf("Failed to create direct subscriber: %v", err)
	}
	defer directSub.Close()

	log.Println()
	log.Println("All subscribers listening on 'chatter'. Press Ctrl+C to exit.")

	// Wait for interrupt signal
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	shutdownCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()

	<-sigChan
	log.Println("Shutting down...")

	<-shutdownCtx.Done()
}
