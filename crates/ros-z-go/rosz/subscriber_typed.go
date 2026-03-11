package rosz

import "sync/atomic"

// TypedMessageHandler is a callback for strongly-typed messages
type TypedMessageHandler[T Message] func(msg T)

// BuildWithTypedCallback creates a subscriber with automatic deserialization.
// The callback receives the already-deserialized message.
//
// Example:
//
//	sub, err := BuildWithTypedCallback(node.CreateSubscriber("chatter"),
//	    func(msg *std_msgs.String) {
//	        log.Printf("Received: %s", msg.Data)
//	    })
func BuildWithTypedCallback[T Message](builder *SubscriberBuilder, handler TypedMessageHandler[T]) (*Subscriber, error) {
	// Create a zero-value instance for type information
	var msgTemplate T

	// Wrap the typed handler with deserialization
	rawHandler := func(data []byte) {
		// Create a new instance of the message type
		var msg T
		if err := msg.DeserializeCDR(data); err != nil {
			// Silently drop malformed messages
			return
		}
		handler(msg)
	}

	return builder.BuildWithCallback(msgTemplate, rawHandler)
}

// SubscriberWithChannel creates a subscriber that delivers to a channel with automatic deserialization.
// Returns the subscriber, a receive channel, and a cleanup function.
// Call the cleanup function to close the channel when done.
//
// Example:
//
//	sub, ch, cleanup, err := SubscriberWithChannel[*std_msgs.String](
//	    node.CreateSubscriber("chatter"), 10)
//	defer cleanup()
//	defer sub.Close()
//
//	for msg := range ch {
//	    log.Printf("Received: %s", msg.Data)
//	}
func SubscriberWithChannel[T Message](builder *SubscriberBuilder, bufferSize int) (*Subscriber, <-chan T, func(), error) {
	var msgTemplate T

	// Create output channel
	outCh := make(chan T, bufferSize)

	// Guard against send-on-closed-channel during teardown
	var closed atomic.Bool

	// Wrap with deserialization
	rawHandler := func(data []byte) {
		if closed.Load() {
			return // Channel closed, discard
		}
		var msg T
		if err := msg.DeserializeCDR(data); err != nil {
			return // Drop malformed messages
		}
		outCh <- msg
	}

	sub, err := builder.BuildWithCallback(msgTemplate, rawHandler)
	if err != nil {
		close(outCh)
		return nil, nil, nil, err
	}

	// Cleanup function marks closed then closes the channel
	cleanup := func() {
		closed.Store(true)
		close(outCh)
	}

	return sub, outCh, cleanup, nil
}

// SubscriberWithHandler integrates the Handler interface directly with the Subscriber.
// Returns the subscriber, receive channel, and a cleanup function.
// Call cleanup when done to close the handler.
//
// Example:
//
//	handler := rosz.NewFifoChannel[*std_msgs.String](10)
//	sub, ch, cleanup, err := SubscriberWithHandler(
//	    node.CreateSubscriber("chatter"), handler)
//	defer cleanup()
//	defer sub.Close()
//
//	for msg := range ch {
//	    log.Printf("Received: %s", msg.Data)
//	}
func SubscriberWithHandler[T Message](builder *SubscriberBuilder, handler Handler[T]) (*Subscriber, <-chan T, func(), error) {
	var msgTemplate T

	callback, drop, ch := handler.ToCbDropHandler()

	// Wrap callback with deserialization
	rawCallback := func(data []byte) {
		var msg T
		if err := msg.DeserializeCDR(data); err != nil {
			return
		}
		callback(msg)
	}

	sub, err := builder.BuildWithCallback(msgTemplate, rawCallback)
	if err != nil {
		if drop != nil {
			drop()
		}
		return nil, nil, nil, err
	}

	return sub, ch, drop, nil
}
