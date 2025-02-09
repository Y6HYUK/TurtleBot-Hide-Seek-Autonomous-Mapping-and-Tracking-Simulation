#!/usr/bin/env python3
import sys
import threading
import subprocess
import os
import random
import time
import math
import yaml
import cv2
import numpy as np
import heapq  # (필요시 사용)

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QTimer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
import rclpy.action
from nav2_msgs.action import NavigateToPose

class SpawnControllerGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'spawn_controller_node')
        QWidget.__init__(self)
        self.setWindowTitle("Spawn Controller GUI")
        self.setMinimumSize(500, 300)
        self.setStyleSheet("""
            QWidget {
                background-color: #f0f0f0;
                font-family: Arial, sans-serif;
                font-size: 14px;
            }
            QPushButton {
                background-color: #007ACC;
                color: white;
                border: none;
                padding: 10px 16px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #005A9E;
            }
            QLabel {
                color: #333333;
            }
        """)

        # 좌표 집합
        self.positions = {
            'waiting': {'x': 1.52878, 'y': -1.65644, 'theta': 0.08052},
            'kitchen': {'x': -0.45619, 'y': 0.56279, 'theta': 0.11338},
            'table_1': {'x': 0.46084, 'y': 1.59266, 'theta': 0.19478},
            'table_2': {'x': 0.44416, 'y': 0.49996, 'theta': -0.14040},
            'table_3': {'x': 0.43981, 'y': -0.57137, 'theta': -0.10710},
            'table_4': {'x': 1.98367, 'y': 2.06235, 'theta': -0.17067},
            'table_5': {'x': 2.07775, 'y': 1.04754, 'theta': -0.12802},
            'table_6': {'x': 1.96393, 'y': -1.16607, 'theta': 0.11646},
            'table_7': {'x': 3.19331, 'y': 2.03429, 'theta': -0.09998},
            'table_8': {'x': 3.10916, 'y': 1.06468, 'theta': 0.09119},
            'table_9': {'x': 3.15339, 'y': -1.04911, 'theta': -0.19493},
        }

        # 상태 라벨
        self.status_label = QLabel("Status: Ready", self)
        
        # Spawn 버튼
        self.spawn_button = QPushButton("Spawn Additional Robot", self)
        self.spawn_button.clicked.connect(self.spawn_robot)
        
        # Autonomous Movement 시작/중지 버튼
        self.start_auto_button = QPushButton("Start Autonomous Movement", self)
        self.start_auto_button.clicked.connect(self.start_autonomous_movement)
        self.start_auto_button.setEnabled(False)
        self.stop_auto_button = QPushButton("Stop Autonomous Movement", self)
        self.stop_auto_button.clicked.connect(self.stop_autonomous_movement)
        self.stop_auto_button.setEnabled(False)

        # 레이아웃 구성
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.spawn_button)
        button_layout.addWidget(self.start_auto_button)
        button_layout.addWidget(self.stop_auto_button)
        
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.status_label)
        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)
        
        self.robot_counter = 2
        self.last_spawned_namespace = None
        self.auto_active = False
        self.auto_thread = None
        
        self.current_pose = None
        self.current_goal = None

        # 글로벌 목표 퍼블리셔를 "/move_base_simple/goal"로 생성 (Nav2가 이 토픽을 구독)
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)

        # 맵 파일 경로 설정 (mapping_gui의 save_map 코드와 동일한 방식)
        self.load_map_files()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_status)
        self.timer.start(500)

    def load_map_files(self):
        # 현재 파일의 절대 경로
        path = os.path.abspath(__file__)
        # "build" 디렉토리가 있으면 "src/my_turtlebot_project"로 대체
        if "build" in path:
            path = path.replace("build" + os.sep + "my_turtlebot_project", "src" + os.sep + "my_turtlebot_project")
        base_dir = os.path.normpath(os.path.join(os.path.dirname(path), ".."))
        maps_dir = os.path.join(base_dir, "maps")
        self.status_label.setText(f"Loading map from: {maps_dir}")
        map_yaml_path = os.path.join(maps_dir, "map.yaml")
        map_image_path = os.path.join(maps_dir, "map.pgm")
        self.load_map(map_yaml_path, map_image_path)

    def load_map(self, yaml_path, image_path):
        try:
            with open(yaml_path, "r") as f:
                map_data = yaml.safe_load(f)
            self.map_resolution = map_data.get("resolution", 0.05)
            origin = map_data.get("origin", [0.0, 0.0, 0.0])
            self.map_origin = (origin[0], origin[1])
        except Exception as e:
            self.get_logger().error("Failed to load map YAML: " + str(e))
            self.map_resolution = 0.05
            self.map_origin = (0.0, 0.0)
        self.map_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if self.map_image is None:
            self.get_logger().error("Failed to load map image at: " + image_path)
        else:
            self.get_logger().info("Map loaded successfully.")

    def update_status(self):
        pass

    def spawn_robot(self):
        self.status_label.setText("Spawning additional robot...")
        threading.Thread(target=self._spawn_robot_command, daemon=True).start()

    def _spawn_robot_command(self):
        entity_name = f"turtlebot3_{self.robot_counter}"
        spawn_x = 0.95 * self.robot_counter
        spawn_y = 0.0
        cmd = (
            f"ros2 run gazebo_ros spawn_entity.py "
            f"-file $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf "
            f"-entity {entity_name} "
            f"-robot_namespace /{entity_name} "
            f"-x {spawn_x} -y {spawn_y} -z 0.01 -Y 0.0 -unpause"
        )
        try:
            subprocess.run(cmd, shell=True, check=True)
            self.status_label.setText(f"Successfully spawned {entity_name}!")
            self.last_spawned_namespace = entity_name
            self.robot_counter += 1
            self.start_auto_button.setEnabled(True)
            self.create_subscription(Odometry, f"/{entity_name}/odom", self.odom_callback, 10)
        except subprocess.CalledProcessError as e:
            self.status_label.setText("Spawn additional robot failed: " + str(e))

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (x, y, yaw)

    def sample_random_goal(self):
        keys = list(self.positions.keys())
        chosen = random.choice(keys)
        pos = self.positions[chosen]
        self.get_logger().info(f"Selected goal: {chosen} -> {pos}")
        return (pos['x'], pos['y'], pos['theta'])

    def start_autonomous_movement(self):
        if not self.last_spawned_namespace:
            self.status_label.setText("No spawned robot to control!")
            return
        self.status_label.setText(f"Starting autonomous movement for {self.last_spawned_namespace}...")
        self.auto_active = True
        self.start_auto_button.setEnabled(False)
        self.stop_auto_button.setEnabled(True)
        self.current_goal = None
        self.auto_thread = threading.Thread(target=self._autonomous_movement_loop, daemon=True)
        self.auto_thread.start()

    def _autonomous_movement_loop(self):
        rate = 1.0  # 1 Hz
        goal_tolerance = 0.1  # m
        while self.auto_active:
            if self.current_pose is None:
                time.sleep(1.0 / rate)
                continue
            if self.current_goal is None:
                self.current_goal = self.sample_random_goal()
                if self.current_goal is None:
                    self.get_logger().warn("Failed to sample a goal from positions.")
                    time.sleep(1.0 / rate)
                    continue
                goal_msg = PoseStamped()
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.header.frame_id = "map"
                goal_msg.pose.position.x = self.current_goal[0]
                goal_msg.pose.position.y = self.current_goal[1]
                goal_msg.pose.position.z = 0.0
                theta = self.current_goal[2]
                qz = math.sin(theta / 2.0)
                qw = math.cos(theta / 2.0)
                goal_msg.pose.orientation.z = qz
                goal_msg.pose.orientation.w = qw
                self.get_logger().info(f"Publishing goal: {self.current_goal}")
                self.goal_pub.publish(goal_msg)
            else:
                cur_x, cur_y, _ = self.current_pose
                distance = math.hypot(self.current_goal[0] - cur_x, self.current_goal[1] - cur_y)
                if distance < goal_tolerance:
                    self.get_logger().info("Goal reached. Sampling new goal...")
                    self.current_goal = None
            time.sleep(1.0 / rate)
        self.get_logger().info("Autonomous movement loop ended.")
        self.status_label.setText(f"Autonomous movement stopped for {self.last_spawned_namespace}.")

    def stop_autonomous_movement(self):
        self.auto_active = False
        self.start_auto_button.setEnabled(True)
        self.stop_auto_button.setEnabled(False)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    spawn_gui = SpawnControllerGUI()
    spawn_gui.show()
    threading.Thread(target=rclpy.spin, args=(spawn_gui,), daemon=True).start()
    app.exec_()
    spawn_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
