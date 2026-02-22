# VHS Tape: Topics Panel
#
# Verifies:
# - Console starts and connects to zenohd
# - /chatter topic is discovered and visible in Topics panel
# - j/k keys navigate the topic list
# - q quits cleanly

Output tests/results/01-topics-panel.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"
Env DIRENV_DISABLE "1"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 4s

# Capture initial state: Topics panel visible, /chatter should appear
Screenshot tests/results/01-topics-initial.png

# Navigate down with j
Type "j"
Sleep 200ms
Screenshot tests/results/01-topics-nav-down.png

# Navigate back up with k
Type "k"
Sleep 200ms
Screenshot tests/results/01-topics-nav-up.png

Type "q"
Sleep 500ms
