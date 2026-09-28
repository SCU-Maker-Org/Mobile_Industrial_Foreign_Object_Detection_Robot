#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光裁剪节点
订阅 /scan，裁掉指定角度范围（单位：度），发布 /scan_filtered

支持多区间，支持跨 0° 的区间（内部自动归一化到 [-180, 180) 判断）
"""

import ast
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        # 参数
        # 裁剪区间（度）：[lo1, hi1, lo2, hi2, ...]
        # 例如裁前 90°：[ -45, 45 ]
        # 例如同时裁前 90° 和后 90°：[ -45, 45, 135, 225 ]
        self.declare_parameter('crop_ranges_deg', [-45.0, 45.0])
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        crop = self.get_parameter('crop_ranges_deg').value

        # 兼容从 launch 传来的字符串形式
        if isinstance(crop, str):
            crop = ast.literal_eval(crop)
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # 解析成区间列表
        self.crop_ranges = []
        for i in range(0, len(crop) - 1, 2):
            lo = float(crop[i])
            hi = float(crop[i + 1])
            self.crop_ranges.append((lo, hi))

        self.get_logger().info(f'裁剪区间(度): {self.crop_ranges}')

        # 传感器 QoS：BEST_EFFORT，和雷达发布端匹配
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(LaserScan, out_topic, sensor_qos)
        self.sub = self.create_subscription(LaserScan, in_topic, self.cb, sensor_qos)

        self.count = 0
        self.cropped_total = 0

    def _normalize(self, deg):
        """归一化到 [-180, 180)"""
        while deg >= 180.0:
            deg -= 360.0
        while deg < -180.0:
            deg += 360.0
        return deg

    def _in_crop(self, deg):
        d = self._normalize(deg)
        for (lo, hi) in self.crop_ranges:
            # 支持跨 0° 的区间：lo > hi 时表示 [lo, 180] ∪ [-180, hi]
            if lo <= hi:
                if lo <= d <= hi:
                    return True
            else:
                if d >= lo or d <= hi:
                    return True
        return False

    def cb(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = msg.intensities
        out.ranges = list(msg.ranges)

        cropped = 0
        for i in range(len(out.ranges)):
            ang_rad = msg.angle_min + i * msg.angle_increment
            ang_deg = math.degrees(ang_rad)
            if self._in_crop(ang_deg):
                if out.ranges[i] != 0.0:
                    out.ranges[i] = 0.0
                    cropped += 1

        self.pub.publish(out)

        self.count += 1
        self.cropped_total += cropped
        if self.count % 200 == 0:
            self.get_logger().info(
                f'已处理 {self.count} 帧, 本帧裁掉 {cropped} 点, '
                f'累计裁掉 {self.cropped_total} 点'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
