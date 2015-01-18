p3dx_launch
===========

ROS package for P3DX robots used on Wroclaw University of Technology
equipped with Hokuyo laser ranger

Paczka ROS-a umożliwiająca autonomiczną nawigację robota Pioneer3DX, wyposażonego w skaner laserowy Hokuyo.

## Instalacja
Należy sklonować zawartość repozytorium do podkatologu src w catkin workspace http://wiki.ros.org/catkin/Tutorials/create_a_workspace
następnie należy paczkę zbudować:
<pre>
/home/lab1_5/catkin_ws$ catkin_make
</pre>

Instalacje niezbędnych paczek ROS-a jest możliwa z wykorzystaniem narzędzia rosdep:
<pre>
rosdep install p3dx_launch
</pre>

