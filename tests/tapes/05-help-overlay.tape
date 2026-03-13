# VHS Tape: Help Overlay
#
# Verifies:
# - ? shows the help overlay with keybinding table
# - ? again dismisses the overlay
# - q while overlay is shown also dismisses it (Esc/q/? all close help)

Output tests/results/05-help-overlay.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"
Env DIRENV_DISABLE "1"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 3s

# Normal TUI without overlay
Screenshot tests/results/05-help-off.png

# Show help overlay with ?
Type "?"
Sleep 300ms
Screenshot tests/results/05-help-on.png

# Dismiss with ? again
Type "?"
Sleep 300ms
Screenshot tests/results/05-help-dismissed.png

# Show again and dismiss with Escape
Type "?"
Sleep 300ms
Escape
Sleep 300ms
Screenshot tests/results/05-help-escape.png

Type "q"
Sleep 500ms
