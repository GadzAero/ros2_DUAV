# *CMD UTILES POUR ROS2*

## PAKAGES
Préparer un pkg python (se metre dans src !) :
 - ros2 pkg create name_to_enter --build-type ament_python

## *COMPILER LE CODE ROS2*
Pour compiler le code :
 - colcon build
Pour sourcer le code :
 - source install/setup.bash
Vous pouvez maintenant lancer vos _nodes_, _launch_, ...
Les 2 d'affilé :
 - colcon build && source install/setup.bash
Pour eviter les pkg longs :
 - colcon build --packages-skip mavlink mavros

## *NODES*
 - ros2 run _pkg node_
 - ros2 node list

## *TOPICS*
 - ros2 topic list
 - ros2 topic echo

## *LAUNCH*
Permet de lancer un ensemble de _nodes_. Attention, les fichiers launch nécéssitent une extension xxx.launch.py pour être reconnu, en plus d'être annoncés dans le _setup.py_.
 - ros2 launch _pgk launch\_file\_name_

## *RQT*
Graph architectural :
 - rqt_graph
Accès à l'outil de manière générale :
 - rqt

## UTILISATION DE NOTRE CODE
 - /home/step/ros2_DUAV/test1.waypoints