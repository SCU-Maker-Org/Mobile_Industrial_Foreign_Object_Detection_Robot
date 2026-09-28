#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 双向串口桥接节点

上行 (STM32 -> ROS2, USART3, 230400):
  - 里程计帧 type=0x01  长度 28
  - IMU 帧   type=0x02  长度 36

下行 (ROS2 -> STM32):
  - /cmd_vel            -> 13 字节帧, type=0x01
  - /tracked/reset_odom -> 13 字节帧, type=0x03

校验：XOR（与 Tracked.c 中 Tracked_Checksum 一致）
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


# ============================================================
# 协议常量（与 Tracked.h 严格一致）
# ============================================================
UP_HEAD0 = 0xAA
UP_HEAD1 = 0x55
UP_TYPE_ODOM = 0x01
UP_TYPE_IMU  = 0x02

CMD_HEAD0       = 0xAA
CMD_HEAD1       = 0x55
CMD_TYPE_VEL    = 0x01
CMD_TYPE_RESET  = 0x03
CMD_TAIL        = 0x0D
CMD_FRAME_LEN   = 13

ODOM_FRAME_LEN = 28
IMU_FRAME_LEN  = 36


def xor_checksum(data: bytes) -> int:
    s = 0
    for b in data:
        s ^= b
    return s


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class STM32Bridge(Node):

    def __init__(self):
        super().__init__('stm32_bridge')

        # -------- 参数 --------
        self.declare_parameter('port', '/dev/motor')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame',  'imu_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame  = self.get_parameter('imu_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        cmd_topic       = self.get_parameter('cmd_vel_topic').value

        # -------- QoS --------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        # -------- 发布者 --------
        self.pub_odom = self.create_publisher(Odometry, 'odom_raw', sensor_qos)
        self.pub_imu  = self.create_publisher(Imu, 'imu/data_raw', sensor_qos)

        # -------- 订阅者 --------
        self.create_subscription(Twist, cmd_topic, self.cmd_vel_callback, 10)

        # -------- 清零 service --------
        self.create_service(Trigger, '/tracked/reset_odom', self.handle_reset_odom)

        # -------- TF --------
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # -------- 串口 --------
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05
            )
            self.get_logger().info(f'串口已打开: {port} @ {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'串口打开失败: {e}')
            raise

        # -------- 状态 --------
        self.buf = bytearray()
        self.running = True
        self.write_lock = threading.Lock()

        self.odom_count = 0
        self.imu_count = 0
        self.err_count = 0

        # -------- 读线程 --------
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # -------- 统计定时器 --------
        self.create_timer(5.0, self.stats_callback)

        self.get_logger().info('=' * 50)
        self.get_logger().info('STM32 Bridge 已启动')
        self.get_logger().info(f'  port       = {port} @ {baudrate}')
        self.get_logger().info(f'  odom_frame = {self.odom_frame}')
        self.get_logger().info(f'  base_frame = {self.base_frame}')
        self.get_logger().info(f'  imu_frame  = {self.imu_frame}')
        self.get_logger().info(f'  publish_tf = {self.publish_tf}')
        self.get_logger().info(f'  cmd_vel    = {cmd_topic}')
        self.get_logger().info('  reset      = /tracked/reset_odom (Trigger)')
        self.get_logger().info('=' * 50)

    # ============================================================
    def rx_loop(self):
        while self.running and rclpy.ok():
            try:
                n = self.ser.in_waiting
                if n > 0:
                    data = self.ser.read(n)
                    self.buf.extend(data)
                    self.parse_buffer()
                else:
                    time.sleep(0.002)
            except serial.SerialException as e:
                self.get_logger().error(f'串口读取异常: {e}')
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f'RX 异常: {e}')
                time.sleep(0.5)

    # ============================================================
    def parse_buffer(self):
        while True:
            if len(self.buf) < 3:
                return
            idx = self.buf.find(bytes([UP_HEAD0, UP_HEAD1]))
            if idx < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                return
            if idx > 0:
                self.buf = self.buf[idx:]
            if len(self.buf) < 3:
                return

            ftype = self.buf[2]
            if ftype == UP_TYPE_ODOM:
                flen = ODOM_FRAME_LEN
            elif ftype == UP_TYPE_IMU:
                flen = IMU_FRAME_LEN
            else:
                self.buf = self.buf[1:]
                continue

            if len(self.buf) < flen:
                return
            frame = bytes(self.buf[:flen])

            if xor_checksum(frame[:-1]) != frame[-1]:
                self.err_count += 1
                self.buf = self.buf[1:]
                continue

            if ftype == UP_TYPE_ODOM:
                self.handle_odom(frame)
            else:
                self.handle_imu(frame)
            self.buf = self.buf[flen:]

    # ============================================================
    def handle_odom(self, frame: bytes):
        (_hdr, _type,
         x, y, yaw, v, w,
         _ts, _chk) = struct.unpack('<2sBfffffIB', frame)

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = float(v)
        odom.twist.twist.angular.z = float(w)

        odom.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]
        odom.twist.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        self.pub_odom.publish(odom)
        self.odom_count += 1

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # ============================================================
    def handle_imu(self, frame: bytes):
        (_hdr, _type,
         gx, gy, gz,
         ax, ay, az,
         yaw,
         _ts, _chk) = struct.unpack('<2sBfffffffIB', frame)

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.imu_frame
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)

        imu.orientation_covariance = [
            1000000.0, 0.0, 0.0,
            0.0, 1000000.0, 0.0,
            0.0, 0.0, 0.001
        ]
        imu.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        imu.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        self.pub_imu.publish(imu)
        self.imu_count += 1

    # ============================================================
    def cmd_vel_callback(self, msg: Twist):
        v = max(min(float(msg.linear.x),  1.0), -1.0)
        w = max(min(float(msg.angular.z), 3.0), -3.0)

        frame = bytearray(CMD_FRAME_LEN)
        frame[0] = CMD_HEAD0
        frame[1] = CMD_HEAD1
        frame[2] = CMD_TYPE_VEL
        struct.pack_into('<f', frame, 3, v)
        struct.pack_into('<f', frame, 7, w)
        frame[11] = xor_checksum(frame[:11])
        frame[12] = CMD_TAIL

        with self.write_lock:
            try:
                self.ser.write(frame)
            except serial.SerialException as e:
                self.get_logger().error(f'串口写入失败: {e}')

    # ============================================================
    def handle_reset_odom(self, request, response):
        """清零 STM32 里程计：type=0x03，格式与 cmd_vel 完全一致"""
        frame = bytearray(CMD_FRAME_LEN)
        frame[0] = CMD_HEAD0
        frame[1] = CMD_HEAD1
        frame[2] = CMD_TYPE_RESET
        # frame[3..10] 保留位 = 0
        frame[11] = xor_checksum(frame[:11])
        frame[12] = CMD_TAIL

        ok = True
        errmsg = 'odom reset sent (x3)'
        with self.write_lock:
            for i in range(3):   # 连发 3 次防丢帧
                try:
                    self.ser.write(frame)
                    self.get_logger().debug(
                        f'  reset frame {i+1}/3: {frame.hex(" ").upper()}'
                    )
                except serial.SerialException as e:
                    ok = False
                    errmsg = str(e)
                    self.get_logger().error(f'清零指令发送失败: {e}')
                    break
                time.sleep(0.05)

        if ok:
            self.get_logger().info(
                f'已发送里程计清零指令 type=0x03 帧: {frame.hex(" ").upper()}'
            )
        response.success = ok
        response.message = errmsg
        return response

    # ============================================================
    def stats_callback(self):
        self.get_logger().info(
            f'[stats] odom={self.odom_count} imu={self.imu_count} err={self.err_count}'
        )

    # ============================================================
    def destroy_node(self):
        self.running = False
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = STM32Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点异常退出: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
