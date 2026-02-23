# VHS Tape: Services Panel
# Switches to the Services panel and browses the available services
# including the AddTwoInts service from z_srvcli

Output scripts/demos/results/06-services.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 3s

# Navigate to Services panel with key '2'
Type "2"
Sleep 500ms
Screenshot scripts/demos/results/06-services-list.png

# Open detail pane for the first service
Type "l"
Sleep 500ms
Screenshot scripts/demos/results/06-services-detail.png

# Return to list
Type "h"
Sleep 300ms

Type "q"
Sleep 500ms
