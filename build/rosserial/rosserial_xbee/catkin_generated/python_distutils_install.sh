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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/hydrosharks2/workspaceRos/src/rosserial/rosserial_xbee"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hydrosharks2/workspaceRos/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hydrosharks2/workspaceRos/install/lib/python2.7/dist-packages:/home/hydrosharks2/workspaceRos/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hydrosharks2/workspaceRos/build" \
    "/usr/bin/python2" \
    "/home/hydrosharks2/workspaceRos/src/rosserial/rosserial_xbee/setup.py" \
     \
    build --build-base "/home/hydrosharks2/workspaceRos/build/rosserial/rosserial_xbee" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hydrosharks2/workspaceRos/install" --install-scripts="/home/hydrosharks2/workspaceRos/install/bin"
