# VHS Tape: Rate Check
#
# Verifies:
# - r key triggers a quick rate measurement on the selected topic
# - Status bar shows measurement in progress
# - After measurement completes, rate (msg/s) is shown in the topic list

Output tests/results/06-rate-check.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 4s

# /chatter should be selected (first topic)
Screenshot tests/results/06-rate-before.png

# Trigger rate measurement with r
Type "r"
Sleep 500ms
Screenshot tests/results/06-rate-measuring.png

# Wait for measurement to complete (default quick measure: ~3s)
Sleep 4s
Screenshot tests/results/06-rate-done.png

Type "q"
Sleep 500ms
