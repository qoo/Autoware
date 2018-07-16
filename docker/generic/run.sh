#!/bin/sh

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_DIR=/home/autoware/shared_dir
HOST_DIR=/home/$USER/shared_dir
xhost +
if [ "$1" = "kinetic" ] || [ "$1" = "indigo" ]
then
    echo "Use $1"
else
    echo "Select distribution, kinetic|indigo"
    exit
fi

if [ "$2" = "" ]
then
    # Create Shared Folder
    mkdir -p $HOST_DIR
else
    HOST_DIR=$2
fi
echo "Shared directory: ${HOST_DIR}"

nvidia-docker run \
    -it --rm \
      -e SHELL\
      -e DISPLAY=unix:0.0\
      -e NO_AT_BRIDGE=1 \
      -e QT_X11_NO_MITSHM=1\
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    -u autoware \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    autoware-$1
