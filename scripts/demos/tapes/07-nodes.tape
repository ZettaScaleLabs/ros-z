# VHS Tape: Nodes Panel
# Switches to the Nodes panel, shows discovered nodes (talker, listener, srvcli),
# then drills into a node to see its associated publishers/subscribers/services

Output scripts/demos/results/07-nodes.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 3s

# Navigate to Nodes panel with key '3'
Type "3"
Sleep 500ms
Screenshot scripts/demos/results/07-nodes-list.png

# Open detail pane for the first node
Type "l"
Sleep 500ms
Screenshot scripts/demos/results/07-nodes-detail.png

# Scroll through the node's topics/services
Type "j"
Sleep 400ms
Type "j"
Sleep 400ms
Screenshot scripts/demos/results/07-nodes-detail-scrolled.png

# Return to list
Type "h"
Sleep 300ms

Type "q"
Sleep 500ms
