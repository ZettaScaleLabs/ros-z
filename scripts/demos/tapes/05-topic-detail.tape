# VHS Tape: Topic Detail
# Selects /chatter in the Topics panel and drills into the detail pane
# showing publishers, subscribers, type hash, and QoS profiles

Output scripts/demos/results/05-topic-detail.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 3s

# Topics panel, /chatter selected
Screenshot scripts/demos/results/05-topic-list.png

# Open detail pane
Type "l"
Sleep 500ms
Screenshot scripts/demos/results/05-topic-detail-top.png

# Navigate down through sections (publishers → subscribers → QoS)
Type "j"
Sleep 400ms
Type "j"
Sleep 400ms
Screenshot scripts/demos/results/05-topic-detail-qos.png

# Return to list
Type "h"
Sleep 300ms

Type "q"
Sleep 500ms
