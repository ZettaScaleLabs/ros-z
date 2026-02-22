# VHS Tape: Panel Navigation
# Demonstrates Tab switching between panels and list/detail pane navigation

Output demos/results/02-navigation.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 3s

# Topics panel (default)
Screenshot demos/results/02-nav-topics.png

# Tab → Services
Tab
Sleep 500ms
Screenshot demos/results/02-nav-services.png

# Tab → Nodes
Tab
Sleep 500ms
Screenshot demos/results/02-nav-nodes.png

# Tab → Measure
Tab
Sleep 500ms
Screenshot demos/results/02-nav-measure.png

# Back to Topics (Shift+Tab × 3)
Shift+Tab
Shift+Tab
Shift+Tab
Sleep 500ms

# Enter detail pane with 'l'
Type "l"
Sleep 500ms
Screenshot demos/results/02-nav-detail.png

# Navigate sections in detail pane (j/k)
Type "j"
Sleep 300ms
Type "k"
Sleep 300ms

# Return to list pane with 'h'
Type "h"
Sleep 500ms
Screenshot demos/results/02-nav-list.png

Type "q"
Sleep 500ms
