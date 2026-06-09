#!/usr/bin/env bash
# pass-through commands to 'docker run' with some defaults
# https://docs.docker.com/engine/reference/commandline/run/
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# check for V4L2 devices
V4L2_DEVICES=""

for i in {0..9}
do
	if [ -a "/dev/video$i" ]; then
		V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
	fi
done

# check for display
DISPLAY_DEVICE=""

if [ -n "$DISPLAY" ]; then
	# give docker root user X11 permissions
	sudo xhost +si:localuser:root
	
	# enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
	XAUTH=/tmp/.docker.xauth
	xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
	chmod 777 $XAUTH

	DISPLAY_DEVICE="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH"
fi

# check if sudo is needed
if id -nG "$USER" | grep -qw "docker"; then
	SUDO=""
else
	SUDO="sudo"
fi

# Foreground by default; DETACH=1 runs the container in the background
# (-dit: the allocated TTY keeps the default bash alive). Enter it with:
#   docker exec -it felix-ai bash
RUN_MODE="-it"
[ "${DETACH:-0}" = "1" ] && RUN_MODE="-dit"

# run the container
ARCH=$(uname -i)

if [ $ARCH = "aarch64" ]; then

	# this file shows what Jetson board is running
	# /proc or /sys files aren't mountable into docker
	cat /proc/device-tree/model > /tmp/nv_jetson_model

	set -x

	#--volume $ROOT/data:/data \
	#$SUDO docker run --runtime nvidia -it --rm --network host -e ROBOT=${ROBOT} \
	$SUDO docker run --runtime nvidia $RUN_MODE --rm \
		--network host --ipc host \
		-e ROBOT=${ROBOT} \
		-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} \
		--name felix-ai \
		--volume /tmp/argus_socket:/tmp/argus_socket \
		--volume /etc/enctune.conf:/etc/enctune.conf \
		--volume /etc/nv_tegra_release:/etc/nv_tegra_release \
		--volume /tmp/nv_jetson_model:/tmp/nv_jetson_model \
		--device /dev/gpiochip0 \
		--device /dev/gpiochip1 \
		--volume ${HOME}/felix-ai:/felix-ai \
		--volume ${HOME}/felix-ai-ros:/felix-ai-ros \
		--volume ${HOME}/data:/data \
		--volume ${HOME}/.claude-felix:/.claude \
		--device /dev/snd \
		--device /dev/bus/usb \
		--device /dev/rplidar \
		--device /dev/myserial \
		--device /dev/mypico \
		--device /dev/i2c-1 \
		--device /dev/i2c-2 \
		--device /dev/i2c-7 \
		$DATA_VOLUME $DISPLAY_DEVICE $V4L2_DEVICES \
		"$@"

		# --device /dev/myserial
		# --device /dev/mypico
		#--device /dev/ttyACM0
		#--device /dev/ttyUSB0

elif [ $ARCH = "x86_64" ]; then

	set -x

	$SUDO docker run --gpus all $RUN_MODE --rm --network=host \
		--shm-size=8g \
		--ulimit memlock=-1 \
		--ulimit stack=67108864 \
		--env NVIDIA_DRIVER_CAPABILITIES=all \
		--volume $ROOT/data:/data \
		$DATA_VOLUME $DISPLAY_DEVICE $V4L2_DEVICES \
		"$@"
fi

if [ "${DETACH:-0}" = "1" ]; then
	set +x
	echo "Container 'felix-ai' started detached."
	echo "  enter:  docker exec -it felix-ai bash"
	echo "  stop:   docker stop felix-ai"
fi
