# VHS Tape: Panel Navigation
#
# Verifies:
# - Number keys 1/2/3/4 switch between Topics/Services/Nodes/Measure panels
# - Tab cycles forward through panels
# - Each panel shows a distinct header

Output tests/results/02-panel-navigation.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"
Env DIRENV_DISABLE "1"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 4s

# Default: Topics panel (panel 1)
Screenshot tests/results/02-topics-panel.png

# Switch to Services panel via number key
Type "2"
Sleep 300ms
Screenshot tests/results/02-services-panel.png

# Switch to Nodes panel
Type "3"
Sleep 300ms
Screenshot tests/results/02-nodes-panel.png

# Switch to Measure panel
Type "4"
Sleep 300ms
Screenshot tests/results/02-measure-panel.png

# Tab cycles forward: Measure -> Topics
Tab
Sleep 300ms
Screenshot tests/results/02-tab-back-to-topics.png

# Return to Topics via key 1
Type "1"
Sleep 200ms

Type "q"
Sleep 500ms
