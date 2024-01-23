#!bin/bash

source devel/setup.bash

gnome-terminal --tab --title="roscore" -- bash -c "source devel/setup.bash; sleep 3s ; roscore; exec bash"
gnome-terminal --tab --title="fourbot" -- bash -c "source devel/setup.bash; sleep 6s ; roslaunch bob realsense_mapping.launch; exec bash"
gnome-terminal --tab --title="camera.py" -- bash -c "cd src/simple_navigation_goals/src; sleep 9s;./basic_goal_publish.py; exec bash"
