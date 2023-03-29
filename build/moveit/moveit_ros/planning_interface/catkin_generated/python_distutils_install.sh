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

echo_and_run cd "/home/sridevi/sawyer_ws/src/moveit/moveit_ros/planning_interface"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sridevi/sawyer_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sridevi/sawyer_ws/install/lib/python3/dist-packages:/home/sridevi/sawyer_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sridevi/sawyer_ws/build" \
    "/usr/bin/python3" \
    "/home/sridevi/sawyer_ws/src/moveit/moveit_ros/planning_interface/setup.py" \
    egg_info --egg-base /home/sridevi/sawyer_ws/build/moveit/moveit_ros/planning_interface \
    build --build-base "/home/sridevi/sawyer_ws/build/moveit/moveit_ros/planning_interface" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sridevi/sawyer_ws/install" --install-scripts="/home/sridevi/sawyer_ws/install/bin"
