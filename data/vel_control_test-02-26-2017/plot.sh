#!/bin/bash

file_prefix=`basename $1 .bag`

rostopic echo -b ${file_prefix}.bag -p /beluga/navigation/odometry/filtered > ${file_prefix}_ekf.csv
rostopic echo -b ${file_prefix}.bag -p /beluga/control_inputs/left > ${file_prefix}_left_input.csv
rostopic echo -b ${file_prefix}.bag -p /beluga/control_inputs/right > ${file_prefix}_right_input.csv
rostopic echo -b ${file_prefix}.bag -p /beluga/control_inputs/top > ${file_prefix}_top_input.csv
rostopic echo -b ${file_prefix}.bag -p /beluga/control_inputs/bottom > ${file_prefix}_bottom_input.csv
