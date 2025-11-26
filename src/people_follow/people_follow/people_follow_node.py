import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from ultralytics import YOLO
import cv2
import numpy as np

class PeopleFollow(Node):
    def __init__(self):
        super().__init__('people_follow')
        
        # 参数声明
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model', 'yolov8n.pt'),
                ('input_image_topic', '/camera/image_raw'),
                ('enable_cuda', True),
                ('conf_threshold', 0.5),
                ('target_class', 'person'),
                ('kp_linear', 0.5),
                ('kp_angular', 1.0),
                ('max_linear_speed', 0.3),
                ('max_angular_speed', 1.0),
                ('target_distance', 2.0),
                ('image_center_x', 320),
                ('image_center_y', 240)
            ]
        )
        
        # 参数获取
        model_path = self.get_parameter('model').value
        input_topic = self.get_parameter('input_image_topic').value
        enable_cuda = self.get_parameter('enable_cuda').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.target_class = self.get_parameter('target_class').value
        
        # PID控制参数
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.target_distance = self.get_parameter('target_distance').value
        
        # 图像中心
        self.image_center_x = self.get_parameter('image_center_x').value
        self.image_center_y = self.get_parameter('image_center_y').value
        
        # 初始化YOLO模型
        self.model = YOLO(model_path)
        if enable_cuda:
            self.model.to('cuda')
        self.model.fuse()

        # 图像处理工具
        self.bridge = CvBridge()
        
        # 当前检测到的人体框中心
        self.current_center = None
        self.current_bbox_area = 0
        
        # 订阅/发布
        self.sub = self.create_subscription(Image, input_topic, self.image_callback, 10)
        self.pub_image = self.create_publisher(Image, 'detected_image', 10)
        self.pub_detections = self.create_publisher(Detection2DArray, 'detections', 10)
        self.pub_center = self.create_publisher(Point, 'person_center', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 帧率计算
        self.frame_count = 0
        self.last_time = self.get_clock().now()
        
        # 控制定时器
        self.control_timer = self.create_timer(0.1, self.control_callback)  # 10Hz
        
        self.get_logger().info('People Follow node initialized')
        self.get_logger().info(f'Target class: {self.target_class}')

    def image_callback(self, msg):
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_height, image_width = cv_image.shape[:2]
            
            # 更新图像中心（如果图像尺寸变化）
            self.image_center_x = image_width // 2
            self.image_center_y = image_height // 2
            
            # 执行推理
            results = self.model.predict(
                source=cv_image,
                conf=self.conf_threshold,
                classes=[0],  # 只检测人 (COCO数据集中人的类别ID是0)
                verbose=False
            )

            # 准备检测结果消息
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            # 绘制检测结果
            annotated_image = results[0].plot()
            
            # 计算并显示帧率
            current_time = self.get_clock().now()
            delta_time = current_time - self.last_time
            fps = 1e9 / delta_time.nanoseconds if delta_time.nanoseconds > 0 else 0.0
            cv2.putText(annotated_image, f'FPS: {fps:.2f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 绘制图像中心
            cv2.circle(annotated_image, (self.image_center_x, self.image_center_y), 
                      5, (255, 0, 0), -1)
            
            # 重置当前中心
            self.current_center = None
            self.current_bbox_area = 0
            
            # 填充检测结果并找到最大的人体框
            max_area = 0
            selected_detection = None
            
            for box in results[0].boxes:
                if int(box.cls) == 0:  # 只处理人的检测
                    # 计算边界框面积
                    bbox_width = float(box.xywh[0][2])
                    bbox_height = float(box.xywh[0][3])
                    area = bbox_width * bbox_height
                    
                    # 选择最大的人体框
                    if area > max_area:
                        max_area = area
                        selected_detection = box
                    
                    detection = Detection2D()
                    detection.bbox = BoundingBox2D()
                    detection.bbox.center.position.x = float(box.xywh[0][0])
                    detection.bbox.center.position.y = float(box.xywh[0][1])
                    detection.bbox.size_x = bbox_width
                    detection.bbox.size_y = bbox_height
                    
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = self.model.names[int(box.cls)]
                    hypothesis.hypothesis.score = float(box.conf)
                    detection.results.append(hypothesis)
                    
                    detections_msg.detections.append(detection)
            
            # 处理选中的人体框
            if selected_detection is not None:
                center_x = float(selected_detection.xywh[0][0])
                center_y = float(selected_detection.xywh[0][1])
                bbox_width = float(selected_detection.xywh[0][2])
                bbox_height = float(selected_detection.xywh[0][3])
                
                self.current_center = (center_x, center_y)
                self.current_bbox_area = bbox_width * bbox_height
                
                # 发布人体中心点
                center_msg = Point()
                center_msg.x = center_x
                center_msg.y = center_y
                center_msg.z = self.current_bbox_area  # 使用z坐标传递面积信息
                self.pub_center.publish(center_msg)
                
                # 在图像上绘制中心点和目标点
                cv2.circle(annotated_image, (int(center_x), int(center_y)), 
                          8, (0, 0, 255), -1)
                cv2.line(annotated_image, 
                         (int(center_x), int(center_y)),
                         (self.image_center_x, self.image_center_y),
                         (0, 255, 255), 2)
                
                # 显示控制信息
                error_x = center_x - self.image_center_x
                cv2.putText(annotated_image, f'Error X: {error_x:.1f}', 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(annotated_image, f'Area: {self.current_bbox_area:.0f}', 
                           (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 发布带检测结果的图像
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8'))
            
            # 发布检测结果
            self.pub_detections.publish(detections_msg)
            self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

    def control_callback(self):
        """控制机器人的运动"""
        cmd_vel_msg = Twist()
        
        if self.current_center is not None:
            center_x, center_y = self.current_center
            
            # 计算水平方向误差
            error_x = center_x - self.image_center_x
            
            # 计算角速度（使人体框水平居中）
            angular_z = -self.kp_angular * (error_x / self.image_center_x)
            angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)
            
            # 计算线速度（根据边界框大小调整距离）
            # 面积越大表示人越近，需要后退；面积越小表示人越远，需要前进
            target_area = (self.target_distance * 100) ** 2  # 简化的面积目标
            area_error = target_area - self.current_bbox_area
            linear_x = self.kp_linear * (area_error / target_area)
            linear_x = np.clip(linear_x, -self.max_linear_speed, self.max_linear_speed)
            
            cmd_vel_msg.linear.x = linear_x
            cmd_vel_msg.angular.z = angular_z
            
            self.get_logger().info(f'Control: linear={linear_x:.2f}, angular={angular_z:.2f}')
        
        else:
            # 没有检测到人时停止
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.get_logger().info('No person detected, stopping')
        
        # 发布控制命令
        self.pub_cmd_vel.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PeopleFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        stop_msg = Twist()
        node.pub_cmd_vel.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()