#!/bin/bash

found_dual_bravo=0
found_single_bravo=0

while [ $found_dual_bravo -eq 0 ] && [ $found_single_bravo -eq 0 ]; do
    rosservice info /girona1000/bravo_right/controller_manager/list_controllers &> /dev/null && \
    rosservice info /girona1000/bravo_left/controller_manager/list_controllers &> /dev/null
    if [ $? -eq 0 ]; then
        found_dual_bravo=1
    fi

    rosservice info /girona500/bravo/controller_manager/list_controllers &> /dev/null
    if [ $? -eq 0 ]; then
        found_single_bravo=1
    fi

    sleep 1
done

sleep 10

if [ $found_dual_bravo -eq 1 ]; then
    echo "Found dual bravo"
    rosservice call /girona1000/bravo_left/predefined_motions/do "data: 'fold'" &
    rosservice call /girona1000/bravo_right/predefined_motions/do "data: 'fold'"
else
    echo "Found single bravo"
    rosservice call /girona500/bravo/predefined_motions/do "data: 'fold'"
fi
