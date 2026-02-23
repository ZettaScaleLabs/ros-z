# ros-z-console TUI Demos

These demos use the classic `z_pubsub` (talker/listener) and `z_srvcli` (AddTwoInts server)
examples as a live ROS 2 system.

**Setup:**

```bash
# Terminal 1
zenohd

# Terminal 2
ros-z-console tcp/127.0.0.1:7447 0
```

---

## Startup

ros-z-console connects to the Zenoh router and discovers all live entities in the graph.

![ros-z-console connecting to a ROS 2 system and discovering topics and nodes](../img/01-startup.gif)

---

## Topics Panel

The Topics panel (default) lists all active ROS 2 topics with their type and publisher/subscriber counts.
Navigate with `j` / `k` (or arrow keys). Press `l` or `Enter` to open the detail pane.

![Browsing the topics list in ros-z-console](../img/02-navigation.gif)

---

## Topic Detail

Select a topic and press `l` or `Enter` to view publishers, subscribers, type hash, and QoS profiles.

![Topic detail view showing publishers, subscribers and QoS](../img/05-topic-detail.gif)

---

## Services Panel

Press `2` (or `Tab`) to switch to the Services panel. Lists all active ROS 2 services with their type.

![Services panel listing active ROS 2 services](../img/06-services.gif)

---

## Nodes Panel

Press `3` to switch to the Nodes panel. Lists all active nodes. Select a node and press `l` or `Enter`
to see its publishers, subscribers, and services.

![Nodes panel with per-node topic and service associations](../img/07-nodes.gif)

---

## Rate Measurement

Press `r` on a selected topic for a quick rate check (cached 30s). Switch to the Measure panel
(`4` or `m`) for a continuous measurement with a 60-second time-series chart.

![Rate measurement and time-series chart in ros-z-console](../img/08-measurement.gif)

---

## Filter Mode

Press `/` to enter filter mode and start typing. The list narrows to matching items in real time.
Press `Ctrl+U` to clear, `Escape` to exit filter mode.

![Filter mode with type-ahead search in ros-z-console](../img/03-filter.gif)

---

## Help Overlay

Press `?` to toggle the help overlay showing all keybindings.

![Help overlay showing all keybindings in ros-z-console](../img/04-help.gif)
