#!/usr/bin/env python
""" Spawn Agnathax """

import time
import roslaunch


def spawn_node(islandNamespace=None, namespace=None, island_id=None, robot_id=None, model_location=None, camera_location=None):
    """ Launch simulation with Gazebo """
    
    # print(model_location)

    arg_model = "-param robot_description -urdf -model LIS_{}_{} -x {} -y {} -z {}".format(island_id, robot_id, model_location[0], model_location[1], model_location[2])
    # print(arg_model + ": ARG MODEL")
    
    arg_camera = "-urdf -param camera -model camera_{}_{} -x {} -y {} -z {} ".format(island_id, robot_id, (camera_location[0]), (camera_location[1] - 1), (camera_location[2]))
    # print(arg_camera + ": ARG CAMERA")
    
    arg_gazebo = " -gazebo_namespace {}".format(islandNamespace) + "gzserver"
    # print(arg_gazebo)

    # gazebo_env_args = "$GAZEBO_MASTER_IP:1134{}".format(4+island_id)

    node_agnathax_spawn = roslaunch.core.Node(
            package="gazebo_ros", 
            node_type="spawn_model", 
            name="spawn_urdf",
            namespace=namespace,
            output="screen", 
            args=(
                arg_model
                + arg_gazebo
            )
        )
    
    node_camera_spawn = roslaunch.core.Node(
            package="gazebo_ros",
            node_type="spawn_model",
            name="spawn_camera",
            namespace=namespace, 
            output="screen",
            args=(
                arg_camera
                + arg_gazebo
            )
        )

    nodes = [
        node_agnathax_spawn,
        node_camera_spawn,
    ]
    return nodes


def spawn():
    """ Launch simulation with Gazebo """
    node_spawns = spawn_node(namespace="/inverted_pendulum/", robot_id=1)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    processes = [launch.launch(node_spawn) for node_spawn in node_spawns]
    return processes


def main():
    """ Main """
    processes = spawn()
    # Loop
    while any([process.is_alive() for process in processes]):
        time.sleep(2)
    # Close
    for process in processes:
        process.stop()
    return


if __name__ == '__main__':
    main()
