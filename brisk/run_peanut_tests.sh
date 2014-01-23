#!/bin/bash

REDWOOD_WS=~/workspace/redwood_ws/

cd $REDWOOD_WS
. RedwoodInternal/Redwood/setup.sh
redwood_make --platform=peanut --no-deps agast
p_redwood_make --platform=peanut --no-deps brisk --catkin-make-args run_tests

#Test data:
cd $REDWOOD_WS/build_peanut/brisk
adb push test_data/ /test/test_data/

#Binaries:
cd $REDWOOD_WS/devel_peanut/bin

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

