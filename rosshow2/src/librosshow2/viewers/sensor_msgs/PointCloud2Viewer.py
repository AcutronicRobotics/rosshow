#!/usr/bin/env python3

import math
import struct
import time
import numpy as np
from sensor_msgs.msg import PointField
import librosshow2.termgraphics as termgraphics

class PointCloud2Viewer(object):
    def __init__(self, canvas, title = ""):
        self.g = canvas
        self.scale = 500.0
        self.spin = 0.0
        self.tilt = np.pi / 3
        self.camera_distance = 50.0
        self.target_scale = self.scale
        self.target_spin = self.spin
        self.target_tilt = self.tilt
        self.target_camera_distance = self.camera_distance
        self.target_time = 0
        self.calculate_rotation()
        self.msg = None
        self.last_update_shape_time = 0
        self.title = title

        # Taken from ROS1 sensor_msgs/point_cloud2.py
        self._DATATYPES = {}
        self._DATATYPES[PointField.INT8]    = ('b', 1)
        self._DATATYPES[PointField.UINT8]   = ('B', 1)
        self._DATATYPES[PointField.INT16]   = ('h', 2)
        self._DATATYPES[PointField.UINT16]  = ('H', 2)
        self._DATATYPES[PointField.INT32]   = ('i', 4)
        self._DATATYPES[PointField.UINT32]  = ('I', 4)
        self._DATATYPES[PointField.FLOAT32] = ('f', 4)
        self._DATATYPES[PointField.FLOAT64] = ('d', 8)

    # Taken from ROS1 sensor_msgs/point_cloud2.py
    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        fmt = '>' if is_bigendian else '<'

        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in self._DATATYPES:
                print('Skipping unknown PointField datatype [%d]' % field.datatype)
            else:
                datatype_fmt, datatype_length = self._DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length

        return fmt

    # Taken from ROS1 sensor_msgs/point_cloud2.py
    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, bytes(cloud.data), math.isnan
        unpack_from = struct.Struct(fmt).unpack_from

        if skip_nans:
            if uvs:
                for u, v in uvs:
                    p = unpack_from(data, (row_step * v) + (point_step * u))
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        p = unpack_from(data, offset)
                        has_nan = False
                        for pv in p:
                            if isnan(pv):
                                has_nan = True
                                break
                        if not has_nan:
                            yield p
                        offset += point_step
        else:
            if uvs:
                for u, v in uvs:
                    yield unpack_from(data, (row_step * v) + (point_step * u))
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        yield unpack_from(data, offset)
                        offset += point_step

    def keypress(self, c):
        if c == "+" or c == "=":
            self.target_camera_distance /= 1.5
        elif c == "-":
            self.target_camera_distance *= 1.5
        elif c == "[":
            self.target_scale /= 1.5
        elif c == "]" or c == "=":
            self.target_scale *= 1.5
        elif c == "left":
            self.target_spin -= 0.1
        elif c == "right":
            self.target_spin += 0.1
        elif c == "down":
            self.target_tilt -= 0.1
        elif c == "up":
            self.target_tilt += 0.1

        self.target_time = time.time()

        self.calculate_rotation()

    def calculate_rotation(self):
        rotation_spin = \
          np.array([[np.cos(self.spin), -np.sin(self.spin), 0],
                    [np.sin(self.spin), np.cos(self.spin), 0],
                    [0, 0, 1]], dtype = np.float16)

        rotation_tilt = \
          np.array([[1, 0, 0],
                    [0, np.cos(self.tilt), -np.sin(self.tilt)],
                    [0, np.sin(self.tilt), np.cos(self.tilt)]], dtype = np.float16)

        self.rotation = np.matmul(rotation_tilt, rotation_spin)

    def update(self, msg):
        self.msg = msg

    def draw(self):
        if not self.msg:
            return

        t = time.time()

        # capture changes in terminal shape at least every 0.25s
        if t - self.last_update_shape_time > 0.25:
            self.g.update_shape()
            self.last_update_shape_time = t

        # animation over 0.5s when zooming in/out
        if self.scale != self.target_scale or self.tilt != self.target_tilt or self.spin != self.target_spin or self.camera_distance != self.target_camera_distance:
            animation_fraction = (time.time() - self.target_time) / 1.0
            if animation_fraction > 1.0:
                self.scale = self.target_scale
                self.tilt = self.target_tilt
                self.spin = self.target_spin
                self.camera_distance = self.target_camera_distance
            else:
                self.scale = (1 - animation_fraction) * self.scale + animation_fraction * self.target_scale
                self.tilt = (1 - animation_fraction) * self.tilt + animation_fraction * self.target_tilt
                self.spin = (1 - animation_fraction) * self.spin + animation_fraction * self.target_spin
                self.camera_distance = (1 - animation_fraction) * self.camera_distance + animation_fraction * self.target_camera_distance
            self.calculate_rotation()

        points = np.array(list(self.read_points(self.msg, skip_nans = True, field_names = ("x", "y", "z"))), dtype = np.float16)
        self.g.clear()
        w = self.g.shape[0]
        h = self.g.shape[1]
        xmax = self.scale
        ymax = xmax * h/w

        # the xyz coordinates rotated to the camera's frame
        rot_points = np.matmul(self.rotation, points.T).T.astype(np.float32)

        # cutoff points less than 1m from camera
        where_visible = rot_points[:,2] > -self.camera_distance + 1.0 

        # points in front of camera (throw away points behind)
        rot_points_visible = rot_points[where_visible, :]
        points_visible = points[where_visible, :]
        rot_points_visible[:,0] /= rot_points_visible[:,2] + self.camera_distance
        rot_points_visible[:,1] /= rot_points_visible[:,2] + self.camera_distance

        # compute screen coordinates
        screen_is = ((0.5 * w + rot_points_visible[:,0] * self.scale)).astype(np.int16)
        screen_js = ((0.5 * h - rot_points_visible[:,1] * self.scale)).astype(np.int16)

        # compute display colors
        screen_c = np.clip((255.0 / 8 * (points_visible[:,2] + 5)), 0.0, 255.0).astype(np.uint8)
        screen_c = np.vstack((255 - screen_c, screen_c * 0, screen_c)).T

        # filter for only points on-screen
        where_valid = (screen_is > 0) & (screen_js > 0) & (screen_is < w) & (screen_js < h)
        screen_is = screen_is[where_valid]
        screen_js = screen_js[where_valid]
        screen_c = screen_c[where_valid, :]

        # display it
        self.g.set_color((255, 255, 255))
        points = np.vstack((screen_is, screen_js)).T
        self.g.points(points, colors = screen_c)

        self.g.set_color((0, 127, 255))
        self.g.text(self.title, (0, self.g.shape[1] - 4))

        self.g.set_color((127, 127, 127))
        self.g.text("up/down: tilt   left/right: rotate   +/-: zoom", (int(self.g.shape[0]/3), self.g.shape[1] - 4))

        self.g.draw()
