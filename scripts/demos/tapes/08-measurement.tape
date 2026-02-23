# VHS Tape: Rate Measurement
# Shows the rate measurement workflow:
#   r  → quick rate check on selected topic (cached result in topic list)
#   m  → open Measure panel for continuous measurement with time-series chart

Output scripts/demos/results/08-measurement.gif
Set FontSize 14
Set Width 1200
Set Height 800
Set Shell nu
Set Theme "Catppuccin Mocha"

Type "./target/debug/ros-z-console tcp/127.0.0.1:7447 0"
Enter
Sleep 3s

# Topics panel with /chatter selected
Screenshot scripts/demos/results/08-before.png

# Quick rate check with 'r'
Type "r"
Sleep 4s
Screenshot scripts/demos/results/08-rate-result.png

# Open Measure panel with 'm' for continuous measurement
Type "m"
Sleep 500ms
Screenshot scripts/demos/results/08-measure-panel.png

# Let the time-series chart accumulate data
Sleep 5s
Screenshot scripts/demos/results/08-measure-chart.png

Type "q"
Sleep 500ms
