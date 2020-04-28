#!/usr/bin/env python
""" Run lip Simulation """

import time
import rospy
import rospkg
import roslaunch
import subprocess
import os
from urdf_parser_py import urdf

from spawn import spawn_node
from gen_camera_urdf import generate_robot as cam_generate_robot
from generate_urdf import get_continous_joints, generate_robot, generate_urdf, get_all_continuous_joints_names


def setupRobots(islandNamespace, robotNamespace, island_id, robot_id):
  """ Setup nodes and params """
  islandNs = islandNamespace + "_{}/".format(island_id)
  island = "island_{}".format(island_id)
  namespace = islandNs + robotNamespace + "_{}/".format(robot_id)

  rospy.set_param("namespace_{}_{}".format(island_id, robot_id), namespace)

  # rospack = rospkg.RosPack()
  # rospack.list()

  robot = generate_robot(namespace)

  robot_xml = robot.to_xml_string()
  print robot_xml
  robot_parsed = urdf.Robot.from_xml_string(robot_xml)
  robot_parsed_xml = robot_parsed.to_xml_string()
  print robot_parsed_xml

  rospy.set_param(namespace + "robot_description", robot_parsed_xml)

  # log_path = rospack.get_path('lip_analysis')

  # rospy.set_param(
  #     "log_path",
  #     log_path
  # )

  joints = get_continous_joints(robot)

  n_joints = 0
  for joint_i, joint in enumerate(joints):
    rospy.set_param(namespace + "joint/{}".format((joint_i)), joint.name)
    n_joints += 1

  print(n_joints)
  arg = namespace + "n_joints"

  rospy.set_param(arg, int(n_joints))

  cam_robot = cam_generate_robot(namespace)
  rospy.set_param(namespace + "camera_description", cam_robot.to_xml_string())
  rospy.set_param(
    namespace + "parameters/control/joints",
    get_all_continuous_joints_names(robot)
  )
  print(get_continous_joints)

  node_state = roslaunch.core.Node(
    package="robot_state_publisher",
    node_type="state_publisher",
    name="robot_state_publisher",
    namespace=namespace,
    machine_name=island
  )

  spawnNodes = spawn_node(
    islandNamespace=islandNs,
    namespace=namespace,
    island_id=island_id,
    robot_id=robot_id,
    model_location=[0,
                    robot_id - 1,
                    0],
    camera_location=[1.2,
                     (robot_id),
                     0.2]
  )
  # robot node
  spawnRobotNode = spawnNodes[0]

  # camera node
  # spawnCameraNode = spawnNodes[1]

  nodes = [
    spawnRobotNode,
    # spawnCameraNode,
    node_state
  ]
  return nodes


def setupIslands(islandNamespace, island_id, robots):
  """ Setup nodes and params """
  nodes = []
  # Load model pramaters
  namespace = islandNamespace + "_{}/".format(island_id)

  island = "{}_{}".format("island", island_id)

  gazebo_env_args = "http://localhost:1134{}".format(4 + island_id)
  ros_gazebo_env_args = "/{}".format(island)

  gz_URI = 11344 + island_id

  rospy.set_param(ros_gazebo_env_args + "URI", gz_URI)

  arg_gazebo = "use_sim_time:=true"
  node_gzserver = roslaunch.core.Node(
      package="gazebo_ros",
      node_type="gzserver",
      name="gzserver",
      respawn=False,
      namespace=namespace,
      # machine_name=island,
      args="worlds/empty.world --verbose {}".format(arg_gazebo),  # switch to bullet if shared_msgs can be used
      env_args=[("GAZEBO_MASTER_URI", gazebo_env_args),
                ("ROS_NAMESPACE", ros_gazebo_env_args)])
  node_gzclient = roslaunch.core.Node(
    package="gazebo_ros",
    node_type="gzclient",
    name="gzclient",
    namespace=namespace,
    respawn=True,
    # machine_name=island,
    env_args=[
      ("GAZEBO_MASTER_URI",
       gazebo_env_args),
      ("ROS_NAMESPACE",
       ros_gazebo_env_args)
    ]
  )
  arg_gazebo = "call --wait {}/use_sim_time".format(namespace + "gzserver")
  print arg_gazebo

  node_gzstate = roslaunch.core.Node(
    # rosservice call gazebo/pause_physics
    package="rosservice",
    node_type="rosservice",
    name="gazebo_properties",
    namespace=namespace,
    args=(arg_gazebo)
  )
  nodes.append(node_gzserver)
  nodes.append(node_gzclient)
  # nodes.append(node_gzstate)

  rospy.set_param(namespace + "robots", robots)

  for robot_id in range(1, robots + 1):
    nodes.extend(setupRobots(islandNamespace, "robot", island_id, robot_id))

  return nodes


def launch_sim(islands, robots):
  """ Launch simulation with Gazebo """
  nodes = []
  processes = []
  launch = roslaunch.scriptapi.ROSLaunch()
  launch.start()

  rospy.set_param("islands", islands)

  for island_id in range(1, islands + 1):
    robot = robots[island_id - 1]
    nodes.extend(setupIslands("/island", island_id, robot))

  for node in nodes:
    print(node.to_xml())

  processes = [launch.launch(node) for node in nodes]

  # Loop
  while any([process.is_alive() for process in processes]):
    time.sleep(2)
  # Close
  for process in processes:
    process.stop()
  return


def main():
  """ Main """

  # @param1 number of islands
  # @param2 number of robots per island
  # @example: launch_sim(2, [1, 2]) : island 1 has 1 model and island 2 has 2 models

  islands = 1
  robots = [1]

  launch_sim(islands, robots)
  return


if __name__ == '__main__':
  main()
