#!/usr/bin/env python
""" A file for generating inverted-pendulum_control.yaml """

import time

import rospy
import roslaunch
# import yaml


def generate_controllers(joints):
    """ Generate inverted-pendulum_control.yaml file """
    d = {}
    state_controller = {
        "joint_state_controller": {
            "type": "joint_state_controller/JointStateController",
            "publish_rate": 1000
        }
    }
    controllers = [
        {
            "type": "effort_controllers/JointEffortController",
            "joint": joint
        }
        for joint in joints
    ]
    position_controllers = {
        "joint_effort_controller_{}".format(joints[i]): controller
        for i, controller in enumerate(controllers)
    }
    d.update(state_controller)
    d.update(position_controllers)

    return d


def run_controllers(namespace):
    """ Run ROS controllers """
    joints = rospy.get_param(namespace + "parameters/control/joints/continuous")
    controllers_yaml = generate_controllers(joints)
    # print(controllers_yaml)
    rospy.set_param(namespace+"control/config", controllers_yaml)
    add = namespace + "control/config/"
    node_control = roslaunch.core.Node(
        package="controller_manager",
        node_type="spawner",
        name="controller_spawner",
        namespace=namespace,
        respawn=False,
        output="screen",
        args=(
            add+"joint_state_controller "
            + " ".join([
                add+"joint_effort_controller_"+joint
                for joint in joints
            ])
        )
    )
    nodes = [node_control]
    return nodes


def main():
    """ Main """

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    nodes = []
    islands = rospy.get_param("islands")
    for island in range(1, 1+islands):
        robots = rospy.get_param("/island_{}/robots".format( island ))
        for robot_id in range(1, 1+robots):
            namespace = rospy.get_param(
                "namespace_{}_{}".format(island, robot_id)
            )
            nodes.extend(run_controllers(namespace))
    
    processes = [launch.launch(n) for n in nodes]
    processes = processes
    # Loop
    while any([process.is_alive() for process in processes]):
        time.sleep(2)
    # Close
    for process in processes:
        process.stop()
    return


if __name__ == "__main__":
    main()
