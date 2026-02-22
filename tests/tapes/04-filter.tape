# VHS Tape: Filter Mode
#
# Verifies:
# - / activates filter mode (status bar changes)
# - Typing narrows the list to matching topics
# - Ctrl+U clears the filter text
# - Escape exits filter mode, restoring full list

Output tests/results/04-filter.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 4s

# Full topic list visible
Screenshot tests/results/04-filter-before.png

# Enter filter mode with /
Type "/"
Sleep 200ms
Screenshot tests/results/04-filter-active.png

# Type search string - should narrow list to /chatter
Type "chatter"
Sleep 300ms
Screenshot tests/results/04-filter-match.png

# Clear with Ctrl+U
Ctrl+U
Sleep 200ms
Screenshot tests/results/04-filter-cleared.png

# Exit filter mode with Escape
Escape
Sleep 200ms
Screenshot tests/results/04-filter-exited.png

Type "q"
Sleep 500ms
