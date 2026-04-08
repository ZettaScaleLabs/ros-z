# VHS Tape: Filter Mode
# Demonstrates type-ahead search with '/' to narrow down topic list

Output scripts/demos/results/03-filter.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 3s

# Full topic list
Screenshot scripts/demos/results/03-filter-before.png

# Enter filter mode
Type "/"
Sleep 300ms
Screenshot scripts/demos/results/03-filter-mode.png

# Type partial match
Type "chatter"
Sleep 500ms
Screenshot scripts/demos/results/03-filter-match.png

# Clear and try another filter
Ctrl+u
Sleep 300ms
Type "chat"
Sleep 500ms

# Exit filter
Escape
Sleep 800ms
Screenshot scripts/demos/results/03-filter-cleared.png

Type "q"
Sleep 500ms
