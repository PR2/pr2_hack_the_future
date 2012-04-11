#!/bin/bash

cat <<EOF
<launch>
   <arg name="remote"/>
EOF

for s in clear_queue create_program create_user dequeue_program get_my_programs get_output get_program get_programs get_queue login logout queue_program run_program update_program start_queue stop_queue
do
   echo "   <node pkg=\"rosproxy\" type=\"register.py\" name=\"$s\" args=\"service $s \$(arg remote)\"/>"
done
echo "</launch>"
