#!/usr/bin/env python
""" Inverted Pendulum generation """

import rospy
import rospkg

from urdf_parser_py import urdf
import lxml.etree as etree
import yaml

import numpy as np

name = "LIP"
n_links = 2

n_len = 2.0/n_links
n_offset = 0.5/n_links

l_f_rad = 0.05
l_f_len = 2.2

l_link_len = n_len
l_link_rad = 0.05

use_ftSensors = True

def orange():
    col_orange = urdf.Material(
        name="orange",
        color=urdf.Color([1.0, 0.423529411765, 0.0392156862745, 1.0])
    )
    return col_orange

def black():
    col_black = urdf.Material(
        name="black",
        color=urdf.Color([0, 0, 0, 1.0])
    )
    return col_black

def frame_inertial():
    """ Return default inertial """
    return urdf.Inertial(
        mass=1,
        inertia=urdf.Inertia(ixx=1, iyy=1, izz=1),
        origin=urdf.Pose(
            xyz=[0, 0, l_f_len/2], 
            rpy=[0, 0, 0]
        )
    )

def frame_collision(geometry=None, origin=None):
    """ Return default collision """
    if geometry is None:
        geometry = urdf.Cylinder(radius=0.05, length=2.)
    if origin is None:
        origin = urdf.Pose(
            xyz=[0, 0, l_f_len/2], 
            rpy=[0, 0, 0]
        )
    return urdf.Collision(
        geometry=geometry,
        origin=origin
    )

def frame_visual(geometry=None, origin=None):
    if geometry is None:
        geometry = urdf.Cylinder(
            radius=l_f_rad,
            length=l_f_len
        )
    if origin is None:
        origin = urdf.Pose(
            xyz=[0, 0, l_f_len/2],
            rpy=[0, 0, 0]
        )
    return urdf.Visual(
        geometry=geometry,
        origin=origin,
        material=urdf.Material(
            name="Gazebo/Orange"
        )
    )

def default_inertial():
    """ Return default inertial """
    return urdf.Inertial(
        mass=1,
        inertia=urdf.Inertia(ixx=1, iyy=1, izz=1),
        origin=urdf.Pose(
            xyz=[0, 0, n_len/2], 
            rpy=[0, 0, 0]
        )
    )

def default_collision(geometry=None, origin=None):
    """ Return default collision """
    if geometry is None:
        geometry = urdf.Cylinder(
            radius=0.05, 
            length=n_len
        )
    if origin is None:
        origin = urdf.Pose(
            xyz=[0, 0, n_len/2],
            rpy=[0, 0, 0]
        )
    return urdf.Collision(
        geometry=geometry,
        origin=origin
    )

def default_visual(geometry=None, origin=None):
    if geometry is None:
        geometry = urdf.Cylinder(
            radius=0.05,
            length=n_len
        )
    if origin is None:
        origin = urdf.Pose(
            xyz=[0, 0, n_len/2],
            rpy=[0, 0, 0]
        )
    return urdf.Visual(
        geometry=geometry,
        origin=origin,
        material=urdf.Material(
            "Gazebo/Black"
        )
    )

def default_dynamics(damping=None, friction=None):
    if damping is None:
        damping = 0.7
    if friction is None:
        friction = 0.
    return urdf.JointDynamics(
        damping=damping,
        friction=friction
    )

def generate_robot(namespace):
    """ Generate Inverted Pendulum URDF """
    # Robot
    robot = urdf.Robot(name)

    # Base
    l_world = urdf.Link("world")
    robot.add_link(l_world)

    # Frame
    frame_geometry = urdf.Cylinder(
        radius=l_f_rad,
        length=l_f_len
    )

    l_frame = urdf.Link("l_frame")
    l_frame.collision = frame_collision(geometry=frame_geometry)
    l_frame.inertial = frame_inertial()
    l_frame.visual = frame_visual(geometry=frame_geometry)
    robot.add_link(l_frame)

    # World Joint
    j_world = urdf.Joint(
        name="fixed",
        parent="world",
        child="l_frame",
        joint_type="fixed",
        origin=urdf.Pose(
            [0, 0, 0],
            [0, 0, 0]
        )
    )
    robot.add_joint(j_world)

    # Links
    l_link = [urdf.Link("l_{}".format(i)) for i in range(n_links)]
    j_link = [urdf.Joint("j_{}".format(i)) for i in range(n_links)]
    for i in range(n_links):
        l_y = (
            0.1 if i == 0
            else 2*l_link_rad)
        l_z = (
            (l_f_len - 0.05) if i == 0
            else (l_link_len - 0.05)
        )
        l_pos = urdf.Pose(
            xyz=[0, l_y, l_z],
            rpy=[0, 0, 0]
        )
        l_link[i].visual = default_visual()
        l_link[i].inertial = default_inertial()
        l_link[i].collision = default_collision()
        robot.add_link(l_link[i])

        j_link[i] = urdf.Joint(
            name="j_{}".format(i),
            parent=(
                "l_frame" if i == 0
                else "l_{}".format(i - 1)
            ),
            child="l_{}".format(i),
            origin=l_pos,
            joint_type="continuous",
            axis=[0, 1, 0],
            dynamics=default_dynamics()
        )
        robot.add_joint(j_link[i])

    for joint in j_link:
        # Transmission
        t = urdf.Transmission(joint.name+"_trans")
        t.type = "transmission_interface/SimpleTransmission"
        transjoint = urdf.TransmissionJoint(name=joint.name)
        transjoint.add_aggregate(
            "hardwareInterface",
            "EffortJointInterface"
        )
        t.add_aggregate("joint", transjoint)
        actuator = urdf.Actuator(joint.name+"_motor")
        actuator.mechanicalReduction = 1
        t.add_aggregate("actuator", actuator)
        robot.add_aggregate("transmission", t)

    # joint force-torque sensors
    if use_ftSensors:
        joints = get_continous_joints(robot)
        for joint_i, joint in enumerate(joints):
            # Provide feedback
            a = etree.Element("gazebo", {"reference": joint.name})
            b = etree.SubElement(a, "provideFeedback")
            b.text = "true"
            robot.add_aggregate("gazebo", a)
            # Sensor
            a = etree.Element("gazebo")
            b = etree.SubElement(a, "plugin", {
                "name": "force_torque_plugin_" +
                joint.name, "filename": "libgazebo_ros_ft_sensor.so"
            })
            c = etree.SubElement(b, "alwaysOn")
            c.text = "true"
            d = etree.SubElement(b, "updateRate")
            d.text = "100"
            d = etree.SubElement(b, "jointName")
            d.text = joint.name
            d = etree.SubElement(b, "topicName")
            d.text = namespace + "ftSensors/" + joint.name
            robot.add_aggregate("gazebo", a)
    
    # Gazebo Color Plugin for all links
    for link_i, link in enumerate(get_all_links(robot)):
        a = etree.Element("gazebo", {"reference": link.name})
        b = etree.SubElement(a, "material")
        b.text = "Gazebo/Black"
        robot.add_aggregate("gazebo", a)

    # Gazebo Color Plugin for Frame
    a = etree.Element("gazebo", {"reference": "l_frame"})
    b = etree.SubElement(a, "material")
    b.text = "Gazebo/Orange"
    robot.add_aggregate("gazebo", a)

    # Gazebo plugin
    a = etree.Element("gazebo")
    b = etree.SubElement(a, "plugin", {
        "name": "gazebo_ros_control",
        "filename": "libgazebo_ros_control.so"
    })
    c = etree.SubElement(b, "robotNamespace")
    c.text = namespace
    # d = etree.SubElement(b, "robotParam")
    # d.text = "/robot_description"
    robot.add_aggregate("gazebo", a)

    return robot

def generate_urdf(robot):
    """ Save robot URDF file """
    xml_text = robot.to_xml_string()
    print(xml_text)
    with open("inverted-pendulum.urdf", "w+") as urdf_file:
        urdf_file.write(xml_text)
    return

def get_continous_joints(robot):
    """ Get robot revolute joints """
    return [joint for joint in robot.joints if joint.type == "continuous"]

def get_all_continuous_joints_names(robot):
    """ Get all joints of the robot """
    typ = "continuous"
    joints = {
        typ: [joint.name for joint in robot.joints if joint.type == "continuous"]
    }
    return joints

def get_all_links(robot):
    """ Get all link names of the robot """
    return [link for link in robot.links if link.name != "l_frame"]

def main():
    """ Main """
    robot = generate_robot("inverted_pendulum")
    generate_urdf(robot)
    return

if __name__ == "__main__":
    main()
