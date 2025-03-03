#!/bin/bash




task="MovingDown"
parameters=$(cat <<EOF
    {
        "x":1.0,
        "y":0.6,
        "z":0.1
    }
EOF
)


ros2 action send_goal -f /yasmintask yasmin_action_interfaces/action/YasminTask "
    task: $task
    parameters: '$parameters'
"

