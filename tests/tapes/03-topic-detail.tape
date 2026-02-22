# VHS Tape: Topic Detail Pane
#
# Verifies:
# - l key moves focus to detail pane showing topic info
# - Detail pane shows type name, publisher/subscriber counts, QoS
# - h key returns focus to list pane
# - Enter also enters detail pane

Output tests/results/03-topic-detail.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 4s

# Capture list pane state (left pane focused)
Screenshot tests/results/03-list-pane.png

# Move to detail pane with l
Type "l"
Sleep 300ms
Screenshot tests/results/03-detail-pane.png

# Scroll detail pane with j
Type "j"
Sleep 200ms
Screenshot tests/results/03-detail-scrolled.png

# Return to list pane with h
Type "h"
Sleep 300ms
Screenshot tests/results/03-back-to-list.png

# Enter also enters detail pane
Enter
Sleep 300ms
Screenshot tests/results/03-detail-via-enter.png

# Escape returns to list
Escape
Sleep 300ms

Type "q"
Sleep 500ms
