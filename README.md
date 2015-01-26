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

## Uruchomienie

### Konfiguracja sieciowa
Zakładając, że wizualizacja będzie uruchomiona na innym komputerze niż komputer pokładowy robota
należy odpowiednio skonfigurować ustawienia sieciowe ROS-a.

Polecenia na robocie można uruchamiać bezpośrednio lub przez SSH.

Na robocie:
<pre>
lab1_5@P3-DX-4322:~$ export ROS_IP=adres_ip_robota
lab1_5@P3-DX-4322:~$ export ROS_MASTER_URI=http://adres_ip_robota:11311
</pre>
Na komputerze zewnętrznym:
<pre>
student@lab15:~$ export ROS_IP=adres_ip_komputera
student@lab15:~$ export ROS_MASTER_URI=http://adres_ip_robota:11311
</pre>

### Incjalizacja
W pierwszej kolejności należy uruchomić na robocie oprogramowanie ROSARIA i obsługę lasera hokuyo
<pre>
lab1_5@P3-DX-4322:~$ roslaunch p3dx_launch p3dx.launch
</pre>
Jeśli konfiguracja sieciowa została przeprowadzona poprawnie po wywołaniu polecenia
<pre>
rostopic list
</pre>
na komputerze robota i zewnętrznym powinny być widoczne te same topic
, m.in.:
<pre>
/diagnostics
/hokuyo_node/parameter_descriptions
/hokuyo_node/parameter_updates
/rosaria/battery_recharge_state
/rosaria/battery_state_of_charge
/rosaria/battery_voltage
/rosaria/bumper_state
/rosaria/cmd_vel
/rosaria/motors_state
/rosaria/parameter_descriptions
/rosaria/parameter_updates
/rosaria/pose
/rosaria/sonar
/rosout
/rosout_agg
/scan
/tf
</pre>


### sterowanie robota padem
Na robocie lub komputerze do wizualizacji:
<pre>
roslaunch p3dx.launch teleop_joy.launch
</pre>

### Mapowanie
Aby stworzyć mapę pomieszczenia należy na robocie uruchomić:
<pre>
lab1_5@P3-DX-4322:~$ roslaunch p3dx_launch mapping.launch
</pre>
Efekty mapowania można podjerzeć w programie RViz:
<pre>
student@lab15:~$ rosrun rviz rviz
</pre>
i wybraniu pliku konfiguracyjnego map.rviz
Po utworzeniu mapy należy wywołać polecenie:
<pre>
lab1_5@P3-DX-4322:~$ rosrun map_server map_saver
</pre>

### Autonomiczna nawigacja
Uruchomienie pakietu nawigacji (na robocie):
<pre>
lab1_5@P3-DX-4322:~$ roslaunch p3dx_launch navigation.launch
</pre>
Działanie nawigacji jest możliwe po otwarciu pliku nav.rviz w programie RViz.
W pierwszej kolejności należy oznaczyć na mapie pozycję startową robota.
Dokładny opis wykorzystania RViz-a do nawigowania robota można znaleźć na stronie
[http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack]



## Konfiguracja
Konfiguracja działania autonomicznej jest zawarta w plikach:
base_local_planner_params.yaml	
costmap_common_params.yaml
global_costmap_params.yaml 
local_costmap_params.yaml
Większość z tych parametrów można zmieniać dynamicznie podczas testowania systemu
z wykorzystaniem narzędzia rqt_reconfigure




W przypadku wykorzystania innego kontrolera niż Logitech Gamepad F710 prawdopodobnie będzie 
wymagane dostosowanie parametrów w pliku teleop_joy.launch.


