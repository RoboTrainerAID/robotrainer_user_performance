robotrainer_user_performance
==========================================
how to use:

Zuvor:

    Robotrainer oder rosbag starten

    Pfad laden: RViZ -> Robotrainer editor panel -> load scenario -> set active

    Robotrainer deviation starten: roslaunch robotrainer_deviation robotrainer_deviation.launch 

Laden der Aufgabe yaml

    rosparam load A1.yaml

Laden der Benutzerid und Versuchungsnumber:

    rosparam set /task/user_id B1

    rosparam set /task/try 1

starten mittels roslaunch:

        roslaunch classical_user_performance classical_user_performance.launch

Programm schreibt eine yaml ins Homeverzeichnis wenn die Aufgabe erflogreich abgeschlossen wurde

## ROS Distro Support

|         | Kinetic | Melodic |
|:-------:|:-------:|:-------:|
| Branch  | [`kinetic-devel`](https://gitlab.ipr.kit.edu/$NAMESPACE$/robotrainer_user_performance/tree/kinetic-devel) | 
| Status  | [![build status](https://gitlab.ipr.kit.edu/$NAMESPACE$/robotrainer_user_performance/badges/kinetic-devel/pipeline.svg)](https://gitlab.ipr.kit.edu/$NAMESPACE$/robotrainer_user_performance/commits/kinetic-devel) | |

