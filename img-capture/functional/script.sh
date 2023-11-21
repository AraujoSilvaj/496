#!/bin/bash

printf 'How many cameras are collecting data?\n 1, 2 or 3?\n'

read input

if [ $input = "1" ]; then
	python3.11 /home/robot/gitHub/496/img-capture/functional/camera.py

elif [ $input = "2" ]; then
	python3.11 /home/robot/gitHub/496/img-capture/functional/camera.py & python3.11 /home/robot/gitHub/496/img-capture/functional/camera2.py

elif [ $input = "3" ]; then	
	python3.11 /home/robot/gitHub/496/img-capture/functional/camera.py & python3.11 /home/robot/gitHub/496/img-capture/functional/camera2.py & python3.11 /home/robot/gitHub/496/img-capture/functional/camera3.py
else 
	echo 'none'

fi
