#!/usr/bin/env python3
import os
import glob
import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
import numpy as np

class RGBDDriver(Node):
    def __init__(self):
        super().__init__('rgbd_driver_node')

        # Parámetros compatibles con el estilo del demo mono
        self.declare_parameter('settings_name', 'TUM',
                               ParameterDescriptor(description='Nombre del set de settings (p.ej. TUM)'))
        self.declare_parameter('image_seq', 'sample_tum_rgbd',
                               ParameterDescriptor(description='Ruta a la secuencia RGB-D'))
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic_rgb', '/camera/color/image_raw')
        self.declare_parameter('topic_depth', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        self.settings_name = self.get_parameter('settings_name').get_parameter_value().string_value
        self.seq_dir = self.get_parameter('image_seq').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.topic_rgb = self.get_parameter('topic_rgb').get_parameter_value().string_value
        self.topic_depth = self.get_parameter('topic_depth').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub_rgb = self.create_publisher(Image, self.topic_rgb, 10)
        self.pub_depth = self.create_publisher(Image, self.topic_depth, 10)

        # Carga lista de archivos (TUM-like)
        rgb_dir = os.path.join(self.seq_dir, 'rgb')
        depth_dir = os.path.join(self.seq_dir, 'depth')
        self.rgb_files = sorted(glob.glob(os.path.join(rgb_dir, '*.png')) + glob.glob(os.path.join(rgb_dir, '*.jpg')))
        self.depth_files = sorted(glob.glob(os.path.join(depth_dir, '*.png')))

        if not self.rgb_files or not self.depth_files:
            self.get_logger().error(f'No encontré imágenes en: {rgb_dir} y/o {depth_dir}')
            raise SystemExit

        # Asumimos correspondencia por orden (si tienes associations.txt úsalo aquí para mapear por timestamp)
        self.idx = 0
        self.period = 1.0 / max(1.0, self.fps)
        self.timer = self.create_timer(self.period, self.tick)
        self.get_logger().info(f'RGB-D driver listo. Frames: {min(len(self.rgb_files), len(self.depth_files))} | fps={self.fps}')

    def tick(self):
        if self.idx >= min(len(self.rgb_files), len(self.depth_files)):
            self.get_logger().info('Secuencia terminada.')
            rclpy.shutdown()
            return

        rgb_path = self.rgb_files[self.idx]
        depth_path = self.depth_files[self.idx]

        # Carga imágenes
        rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)  # BGR8
        depth_raw = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # 16UC1 esperado (mm)

        if rgb is None or depth_raw is None:
            self.get_logger().warn(f'No pude leer: {rgb_path} o {depth_path}')
            self.idx += 1
            return

        # Verifica depth dtype
        if depth_raw.dtype != np.uint16:
            # Si viene en 32FC1 metros, conviértelo a uint16 mm para mantener encoding estable
            if depth_raw.dtype == np.float32 or depth_raw.dtype == np.float64:
                depth_mm = (depth_raw * 1000.0).astype(np.uint16)
            else:
                # como fallback, normaliza
                depth_mm = depth_raw.astype(np.uint16)
        else:
            depth_mm = depth_raw

        now = self.get_clock().now().to_msg()

        # Mensaje RGB
        msg_rgb = Image()
        msg_rgb.header = Header()
        msg_rgb.header.stamp = now
        msg_rgb.header.frame_id = self.frame_id
        msg_rgb.height, msg_rgb.width = rgb.shape[:2]
        msg_rgb.encoding = 'bgr8'
        msg_rgb.step = rgb.shape[1] * 3
        msg_rgb.data = rgb.tobytes()

        # Mensaje Depth (16UC1, mm)
        msg_depth = Image()
        msg_depth.header = Header()
        msg_depth.header.stamp = now
        msg_depth.header.frame_id = self.frame_id
        msg_depth.height, msg_depth.width = depth_mm.shape[:2]
        msg_depth.encoding = '16UC1'
        msg_depth.step = depth_mm.shape[1] * 2
        msg_depth.data = depth_mm.tobytes()

        # Publica
        self.pub_rgb.publish(msg_rgb)
        self.pub_depth.publish(msg_depth)

        self.idx += 1

def main():
    rclpy.init()
    node = RGBDDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()