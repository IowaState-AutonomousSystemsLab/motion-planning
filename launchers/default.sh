#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
roslaunch perception perception.launch veh:=$VEHICLE_NAME

roslaunch imu_driver imu_driver.launch veh:=$VEHICLE_NAME

docker -H duck7.local run --name duckiebot-interface -v /data:/data --privileged --network=host -dit --restart unless-stopped -e ROBOT_TYPE=<ROBOT_TYPE> duckietown/duckiebot-interface:daffy-arm32v7
# rosrun controls controls.launch


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
