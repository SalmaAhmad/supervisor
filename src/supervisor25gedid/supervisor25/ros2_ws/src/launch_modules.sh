#!/bin/bash

# Description: This script launches the simple_pure_pursuit and planning_acceleration modules

echo "🔁 Sourcing ROS 2 workspace..."
source ~/ros2_ws/install/setup.bash

echo "🚀 Launching simple_pure_pursuit..."
ros2 launch simple_pure_pursuit simplePurePursuit.launch.py &
PID1=$!

echo "🚀 Launching planning_acceleration..."
ros2 launch planning_acceleration acceleration_launch.py &
PID2=$!

# Wait for both launches to finish or allow user to interrupt
echo "✅ Both modules launched. PIDs: $PID1 (pure pursuit), $PID2 (acceleration)"
echo "Press Ctrl+C to stop..."

# Wait indefinitely until killed
wait $PID1 $PID2
