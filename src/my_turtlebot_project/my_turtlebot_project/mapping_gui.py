#!/usr/bin/env python3
import sys
import threading
import subprocess

import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel

def rclpy_spin(node):
    rclpy.spin(node)

class MappingController(Node):
    def __init__(self):
        super().__init__('mapping_controller')
        # 여기서 매핑 관련 publisher나 service client를 초기화할 수 있습니다.
    
    def start_mapping(self):
        self.get_logger().info("Mapping started")
        # 실제 매핑 시작 로직 (예를 들어, 토픽 퍼블리싱 혹은 서비스 호출)을 구현하세요.
    
    def save_map(self):
        self.get_logger().info("Saving map...")
        # 외부 명령어를 호출하여 map.yaml, map.pgm 저장
        # 예시: nav2_map_server의 map_saver_cli를 실행 (환경에 맞게 수정하세요)
        try:
            subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "/home/yjh/map"],
                           check=True)
            self.get_logger().info("Map saved successfully")
        except subprocess.CalledProcessError as e:
            self.get_logger().error("Failed to save map: " + str(e))

class MappingGUI(QWidget):
    def __init__(self, mapping_controller):
        super().__init__()
        self.mapping_controller = mapping_controller
        self.init_ui()
    
    def init_ui(self):
        self.setWindowTitle("Mapping Controller")
        layout = QVBoxLayout()

        # 상태를 표시할 라벨
        self.status_label = QLabel("Mapping status: Idle")
        layout.addWidget(self.status_label)

        # 매핑 시작 버튼
        self.start_button = QPushButton("Start Mapping")
        self.start_button.clicked.connect(self.start_mapping)
        layout.addWidget(self.start_button)

        # 맵 저장 버튼 (초기에는 비활성화)
        self.save_button = QPushButton("Save Map")
        self.save_button.clicked.connect(self.save_map)
        self.save_button.setEnabled(False)
        layout.addWidget(self.save_button)

        self.setLayout(layout)
    
    def start_mapping(self):
        self.mapping_controller.start_mapping()
        self.status_label.setText("Mapping status: Mapping in progress...")
        # 매핑 시작 후에 맵 저장 버튼 활성화
        self.save_button.setEnabled(True)

    def save_map(self):
        self.mapping_controller.save_map()
        self.status_label.setText("Mapping status: Map saved.")

def main():
    # ROS2 초기화
    rclpy.init()
    mapping_controller = MappingController()

    # rclpy 스핀을 별도 스레드에서 실행하여 GUI가 블록되지 않도록 함
    spin_thread = threading.Thread(target=rclpy_spin, args=(mapping_controller,), daemon=True)
    spin_thread.start()

    # PyQt5 애플리케이션 실행
    app = QApplication(sys.argv)
    gui = MappingGUI(mapping_controller)
    gui.show()
    app.exec_()

    # 애플리케이션 종료 시 ROS2 노드 종료
    mapping_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
