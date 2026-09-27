#!/bin/bash
# Запуск: docker run --rm -e ROS_DOMAIN_ID=87 -v E:/MY-PROJECT/TrackVector:/repo vectra/tram:dev bash /repo/tools/dev/ros_nan_check.sh nan|inf
# Нода tram_estimator + поток с выставкой и одним плохим показанием тележки.
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87}
export ROS_LOCALHOST_ONLY=1
KIND=${1:-nan}
PARAMS=/ws/install/tram_state_estimator/share/tram_state_estimator/config/tram.yaml
ros2 run tram_state_estimator tram_estimator --ros-args --params-file "$PARAMS" > /tmp/node.log 2>&1 &
NODE_PID=$!
python3 -c "import time; time.sleep(4)"
python3 /repo/tools/dev/ros_nan_check.py "$KIND"
python3 -c "import time; time.sleep(1)"
if kill -0 $NODE_PID 2>/dev/null; then echo "NODE ALIVE (pid $NODE_PID)"; else wait $NODE_PID; echo "NODE DEAD, exit code $?"; fi
echo "---- node log (tail) ----"
tail -n 12 /tmp/node.log
kill $NODE_PID 2>/dev/null
