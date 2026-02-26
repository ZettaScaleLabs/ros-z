#include "_cgo_export.h"
#include "ros_z_ffi.h"

ros_z_MessageCallback getSubscriberCallback() {
	return (ros_z_MessageCallback)goSubscriberCallback;
}
