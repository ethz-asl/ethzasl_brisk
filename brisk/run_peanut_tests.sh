#!/bin/bash

REDWOOD_WS=~/workspace/redwood_ws/

cd $REDWOOD_WS
. RedwoodInternal/Redwood/setup.sh
p_redwood_make --platform=peanut --debug brisk --catkin-make-args run_tests

#Test data:
cd $REDWOOD_WS/build_peanut_debug/brisk
adb push test_data/ /test/test_data/

#Binaries:
cd $REDWOOD_WS/devel_peanut_debug/bin

adb push test_downsampling /test/
adb push test_integral_image /test/
adb push test_popcnt /test/
adb push test_serialization /test/
adb push test_binary_equal /test/

adb shell "cd test && ./test_downsampling"
adb shell "cd test && ./test_integral_image"
adb shell "cd test && ./test_popcnt"
adb shell "cd test && ./test_serialization"
adb shell "cd test && ./test_binary_equal"

