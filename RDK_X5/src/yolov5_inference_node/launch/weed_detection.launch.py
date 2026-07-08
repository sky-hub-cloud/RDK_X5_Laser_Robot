# 文件路径: ~/ros2_ws/src/yolov5_inference_node/launch/weed_detection.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动你的推理节点
        Node(
            package='yolov5_inference_node',
            executable='inference_node',
            name='inference_node',
            output='screen',
        ),
        # 启动 web_video_server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',       
            parameters=[{'port': 8080}, {'address': '0.0.0.0'}],
            parameters=[{'max_rate': 10, 'video_encoder': 'vp8'}],
            output='screen',
        ),
    ])
