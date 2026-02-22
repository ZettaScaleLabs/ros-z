# VHS Tape: Startup
# Shows ros-z-console connecting to a Zenoh router and discovering topics/nodes

Output demos/results/01-startup.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

# Launch the console (router started by generate.nu)
Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter

# Wait for initial discovery
Sleep 3s
Screenshot demos/results/01-startup-connected.png

# Wait another cycle to show live updates
Sleep 3s
Screenshot demos/results/01-startup-populated.png

# Quit
Type "q"
Sleep 500ms
