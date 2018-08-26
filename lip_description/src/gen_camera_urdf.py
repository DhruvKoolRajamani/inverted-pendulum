#!/usr/bin/env python

import rospy
import rospkg

from urdf_parser_py import urdf
from lxml import etree
import yaml

import numpy as np

def default_inertial():
    """ Return default inertial """
    return urdf.Inertial(
        mass=1e-5,
        inertia=urdf.Inertia(ixx=1e-6, iyy=1e-6, izz=1e-6),
        origin=urdf.Pose([-1, 0, 0.5], [0, 0, 0])
    )

def default_collision(geometry=None, origin=None):
    """ Return default collision """
    if geometry is None:
        geometry = urdf.Box([0.05, 0.05, 0.05])
    if origin is None:
        origin = urdf.Pose([-1, 0, 0.5], [0, 0, 0])
    return urdf.Collision(
        geometry=geometry,
        origin=origin
    )

def default_visual():
    return urdf.Visual(
        geometry=urdf.Box([0.05, 0.05, 0.05]),
        material=urdf.Material(name="red"),
        origin=urdf.Pose([-1, 0, 0.5], [0, 0, 0])
    )

def generate_robot(namespace):
    robot = urdf.Robot("Camera")
    ns= namespace

    l_base = urdf.Link("base_link")
    l_base.inertial = default_inertial()
    robot.add_link(l_base)

    l_camera = urdf.Link("camera_link")
    l_camera.inertial = default_inertial()
    l_camera.collision = default_collision()
    l_camera.visual = default_visual()
    robot.add_link(l_camera)

    j_camera = urdf.Joint(
        "camera_joint",
        parent="base_link",
        child="camera_link",
        joint_type="fixed",
        axis=[0, 1, 0],
        origin=urdf.Pose([-1, 0, 0.5])
    )
    robot.add_joint(j_camera)

    a = etree.Element("gazebo", {
        "reference" : "camera_link"
    })
    b = etree.SubElement(a, "sensor", {
        "type" : "camera",
        "name" : "camera1"
    })
    c = etree.SubElement(b, "update_rate")
    c.text = "30.0"
    d = etree.SubElement(b, "camera", {
        "name" : "camera_main"
    })
    r = etree.SubElement(d, "horizontal_fov")
    r.text = "1.3962634"
    e = etree.SubElement(d, "image")
    f = etree.SubElement(e, "width")
    f.text = "800"
    g = etree.SubElement(e, "height")
    g.text = "800"
    h = etree.SubElement(e, "format")
    h.text = "R8G8B8"
    s = etree.SubElement(d, "clip")
    t = etree.SubElement(s, "near")
    t.text = "0.02"
    w = etree.SubElement(s, "far")
    w.text = "300"
    i = etree.SubElement(b, "plugin", {
        "name" : "camera_controller",
        "filename" : "libgazebo_ros_camera.so"
    })
    j = etree.SubElement(i, "alwaysOn")
    j.text = "true"
    k = etree.SubElement(i, "updateRate")
    k.text = "0.0"
    l = etree.SubElement(i, "cameraName")
    l.text = "camera1"
    m = etree.SubElement(i, "imageTopicName")
    m.text = ns + "camera/image_raw"
    n = etree.SubElement(i, "cameraInfoTopicName")
    n.text = ns + "camera/info"
    o = etree.SubElement(i, "frameName")
    o.text = "camera_link"
    p = etree.SubElement(a, "static")
    p.text = "true"
    q = etree.SubElement(a,"turnGravityOff")
    q.text = "true"
    
    robot.add_aggregate("gazebo", a)

    return robot

def generate_urdf(robot):
    """ Save robot URDF file """
    xml_text = robot.to_xml_string()
    print(xml_text)
    with open("agnathax_camera.urdf", "w+") as urdf_file:
        urdf_file.write(xml_text)
    return

def main():
    robot = generate_robot(None)
    generate_urdf(robot)
    return

if __name__ == "__main__":
    main()
