# VHS Tape: Help Overlay
# Shows the '?' help overlay with all keybindings

Output demos/results/04-help.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 3s

# Show help overlay
Type "?"
Sleep 500ms
Screenshot demos/results/04-help-overlay.png

# Dismiss with '?'
Type "?"
Sleep 300ms
Screenshot demos/results/04-help-dismissed.png

Type "q"
Sleep 500ms
