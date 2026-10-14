#!/usr/bin/env python3
"""Small fixture for physical attachment/release tests; never a perception input."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def inertial(mass=.1):
    return f'<inertial><mass>{mass}</mass><inertia><ixx>.001</ixx><iyy>.001</iyy><izz>.001</izz></inertia></inertial>'


def controller(joint, topic, initial=0):
    return f'''<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name><topic>{topic}</topic><initial_position>{initial}</initial_position>
      <p_gain>100</p_gain><i_gain>0</i_gain><d_gain>10</d_gain><cmd_max>20</cmd_max><cmd_min>-20</cmd_min>
    </plugin>'''


def main(path, folded_base=False):
    config = json.dumps({'robot_model': 'agri_robot', 'service': '/world/harvest_fixture/harvest/command',
        'tcp_offset': [-.18, 0, 0],
        'fruits': [{'id': 'tomato_0_fruit_0', 'plant_id': 'tomato_0', 'radius': .03, 'ripe': True}]})
    robot = f'''<model name="agri_robot">
      <link name="base_link"><pose>0 0 0 1.57079632679 0 1.57079632679</pose>{inertial()}
        <collision name="floor"><pose>0 .20309 .17 0 0 0</pose><geometry><box><size>.6 .005 .4</size></box></geometry></collision>
      </link>
      <joint name="anchor" type="fixed"><parent>world</parent><child>base_link</child></joint>
      <link name="gripper_base_link"><pose relative_to="base_link">.18 .5 .17 0 0 0</pose>{inertial()}<gravity>false</gravity></link>
      <joint name="gripper_slide" type="prismatic"><parent>base_link</parent><child>gripper_base_link</child>
        <axis><xyz>0 1 0</xyz><limit><lower>-.1</lower><upper>.3</upper><effort>20</effort><velocity>.2</velocity></limit></axis></joint>'''
    for side in ('left', 'right'):
        robot += f'''<link name="gripper_{side}_finger_link"><pose relative_to="gripper_base_link">0 0 0 0 0 0</pose>{inertial(.01)}<gravity>false</gravity></link>
          <joint name="gripper_{side}_finger_joint" type="prismatic"><parent>gripper_base_link</parent><child>gripper_{side}_finger_link</child>
            <axis><xyz>0 0 1</xyz><limit><lower>0</lower><upper>.035</upper><effort>20</effort><velocity>.2</velocity></limit></axis></joint>'''
        robot += controller(f'gripper_{side}_finger_joint', f'/fixture/{side}', .012)
    robot += controller('gripper_slide', '/fixture/slide') + '</model>'
    text = f'''<sdf version="1.10"><world name="harvest_fixture"><gravity>0 0 -9.81</gravity>
      <physics name="physics" type="ignored"><max_step_size>.001</max_step_size></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <model name="tomato_0"><static>true</static><link name="link"/></model>
      <model name="tomato_0_fruit_0"><pose>.17 0 .5 0 0 0</pose><link name="fruit_link">{inertial(.025)}
        <collision name="fruit_collision"><geometry><sphere><radius>.03</radius></sphere></geometry></collision>
      </link></model>
      {robot}
      <plugin filename="libagri_harvest_system.so" name="agri::HarvestSystem"><config>{config}</config></plugin>
    </world></sdf>'''
    if folded_base:
        root = ET.fromstring(text)
        robot = root.find("world/model[@name='agri_robot']")
        base = robot.find("link[@name='base_link']"); base.set('name', 'base_footprint')
        base.remove(base.find('pose'))
        base.find('collision/pose').text = '.17 0 .20309 0 0 0'
        base.find('collision/geometry/box/size').text = '.4 .6 .005'
        robot.find("joint[@name='anchor']/child").text = 'base_footprint'
        robot.find("joint[@name='gripper_slide']/parent").text = 'base_footprint'
        hand = robot.find("link[@name='gripper_base_link']/pose")
        hand.set('relative_to', 'base_footprint')
        hand.text = '.17 .18 .5 1.57079632679 0 1.57079632679'
        text = ET.tostring(root, encoding='unicode')
    Path(path).write_text(text)


if __name__ == '__main__': main(sys.argv[1])
