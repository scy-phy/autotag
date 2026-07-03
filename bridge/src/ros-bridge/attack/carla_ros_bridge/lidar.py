#!/usr/bin/env python

#
# Copyright (c) 2018, Willow Garage, Inc.
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla lidars
"""

import json
import numpy
import time

from carla_ros_bridge.sensor import Sensor, create_cloud
from carla_ros_bridge.attack.attack_manager import AttackManager


from sensor_msgs.msg import PointCloud2, PointField
from rclpy import qos
from std_msgs.msg import String

try:
    from lidar_attack_msgs.msg import AttackConfig 
except:
    print("Attack message config package not initialized")

import numpy as np


class Lidar(Sensor):

    """
    Actor implementation details for lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode, frame_id):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Lidar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)

        self.lidar_publisher = node.new_publisher(PointCloud2,
                                                  self.get_topic_prefix(),
                                                  qos_profile=qos.qos_profile_sensor_data)
        self.listen()
        self.channels = int(self.carla_actor.attributes.get('channels'))
        self._frame_id = frame_id

        #Attack config
        self._attack_msg_received = False
        self.attack_manager = AttackManager()

        try: 
            qos_profile_attack = qos.qos_profile_sensor_data #somehow no others are working here
            self.attack_config_sub = node.new_subscription(
                AttackConfig,
                '/lidar_attack/config',
                self._attack_config_callback,
                qos_profile=qos_profile_attack
            )
        except:
            print("No Lidar attack possible due to missing config message package")

        #For publishing message when attack is active
        self.attack_status_pub = node.new_publisher(
            String,
            "/attack/status",
            qos_profile=qos.qos_profile_sensor_data
        )
        


    def destroy(self):
        super(Lidar, self).destroy()
        self.node.destroy_publisher(self.lidar_publisher)

    # Callback to receive attack configuration messages 
    def _attack_config_callback(self, msg):
        self._attack_msg_received = True
        self.attack_manager.update_config(msg)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        header = self.get_msg_header(frame_id=self._frame_id , timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1)
        ]

        lidar_data = numpy.fromstring(
            bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        
        ring = numpy.empty((0,1), dtype=numpy.uint16)
        
        for i in range(self.channels):
            current_ring_points_count = carla_lidar_measurement.get_point_count(i)
            ring = numpy.vstack((
                ring,
                numpy.full((current_ring_points_count, 1), i, dtype=numpy.uint16)))
        
        lidar_data = numpy.hstack((lidar_data, ring))

        #Attack:
        attack_applied = False
        attack_name = None

        if self._attack_msg_received and self.name == "sensor/lidar/front": #only apply attack for front sensor
            lidar_data, attack_applied, attack_name = self.attack_manager.apply(lidar_data)
        
        
        # we take the opposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data[:, 1] *= -1
        #point_cloud_msg = create_cloud(header, fields, lidar_data)

        #Some extra type conversion is necessary - otherwise attacks break
        cloud_array = np.zeros(lidar_data.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint16)
        ])

        cloud_array['x'] = lidar_data[:, 0]
        cloud_array['y'] = lidar_data[:, 1]
        cloud_array['z'] = lidar_data[:, 2]
        cloud_array['intensity'] = lidar_data[:, 3]
        cloud_array['ring'] = lidar_data[:, 4].astype(np.uint16)

        point_cloud_msg = create_cloud(header, fields, cloud_array)


        #Publish message that attack is active 
        if self._attack_msg_received and attack_name:
                msg = String()

                payload = {
                    "attack_name": attack_name,
                    "timestamp_sec": header.stamp.sec,
                    "timestamp_nanosec": header.stamp.nanosec,
                    "attack_applied": attack_applied
                }

                msg.data = json.dumps(payload)

                self.attack_status_pub.publish(msg)


        self.lidar_publisher.publish(point_cloud_msg)


class SemanticLidar(Sensor):

    """
    Actor implementation details for semantic lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(SemanticLidar, self).__init__(uid=uid,
                                            name=name,
                                            parent=parent,
                                            relative_spawn_pose=relative_spawn_pose,
                                            node=node,
                                            carla_actor=carla_actor,
                                            synchronous_mode=synchronous_mode)

        self.semantic_lidar_publisher = node.new_publisher(
            PointCloud2,
            self.get_topic_prefix(),
            qos_profile=10)
        self.listen()

    def destroy(self):
        super(SemanticLidar, self).destroy()
        self.node.destroy_publisher(self.semantic_lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform a received semantic lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla semantic lidar measurement object
        :type carla_lidar_measurement: carla.SemanticLidarMeasurement
        """
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='CosAngle', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ObjIdx', offset=16, datatype=PointField.UINT32, count=1),
            PointField(name='ObjTag', offset=20, datatype=PointField.UINT32, count=1)
        ]

        lidar_data = numpy.fromstring(bytes(carla_lidar_measurement.raw_data),
                                      dtype=numpy.dtype([
                                          ('x', numpy.float32),
                                          ('y', numpy.float32),
                                          ('z', numpy.float32),
                                          ('CosAngle', numpy.float32),
                                          ('ObjIdx', numpy.uint32),
                                          ('ObjTag', numpy.uint32)
                                      ]))

        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data['y'] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data.tolist())
        self.semantic_lidar_publisher.publish(point_cloud_msg)
