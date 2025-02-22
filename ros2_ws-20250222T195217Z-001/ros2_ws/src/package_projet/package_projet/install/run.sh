#!/bin/bash

pkill -f ros2


python3 client.py > log.txt 2>&1 &

python3 publisher_i2c_rand.py > log.txt 2>&1 &

python3 capteur_presence.py > log.txt 2>&1 &

python3 serveur.py > log.txt 2>&1 &

python3 sescriber_i2c.py > log.txt 2>&1 &

python3 siteoui2.py > log.txt 2>&1 &

python3 suscriber_basededonne.py > log.txt 2>&1 &

python3 testeur_recu.py > log.txt 2>&1 &

