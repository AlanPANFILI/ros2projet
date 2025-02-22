#!/bin/bash


source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash


python3 serveur.py >> output.log 2>&1 &
python3 capteur_presence.py >> output.log 2>&1 &
python3 publisher_i2c_rand.py >> output.log 2>&1 &
python3 site.py >> output.log 2>&1 &
python3 client.py >> output.log 2>&1 &
python3 suscriber_basededonne.py >> output.log 2>&1 &
python3 suscriber_i2c.py >> output.log 2>&1 &
python3 porte.py >> output.log 2>&1 &



