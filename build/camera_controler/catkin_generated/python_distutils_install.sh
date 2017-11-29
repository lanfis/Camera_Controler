#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/adel/Dropbox/Github/Camera_Controler/src/camera_controler"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/adel/Dropbox/Github/Camera_Controler/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/adel/Dropbox/Github/Camera_Controler/install/lib/python2.7/dist-packages:/home/adel/Dropbox/Github/Camera_Controler/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/adel/Dropbox/Github/Camera_Controler/build" \
    "/usr/bin/python" \
    "/home/adel/Dropbox/Github/Camera_Controler/src/camera_controler/setup.py" \
    build --build-base "/home/adel/Dropbox/Github/Camera_Controler/build/camera_controler" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/adel/Dropbox/Github/Camera_Controler/install" --install-scripts="/home/adel/Dropbox/Github/Camera_Controler/install/bin"
