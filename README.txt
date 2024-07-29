To be run on a Docker container.

To build the docker image:
	docker build --rm --tag IMAGE_NAME .
To execute a docker container on a Ubuntu machine (gives X11 display information):
	docker run --device /dev/dri -it --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix IMAGE_NAME

On one container, run:
	ros2 launch turtlefollow turtlefollow.launch.py
On one other other, run:
	ros2 launch turtlefollow mousecontrol.launch.py

Ensure the host has its X11 port open to Docker:
	xhost +local:docker