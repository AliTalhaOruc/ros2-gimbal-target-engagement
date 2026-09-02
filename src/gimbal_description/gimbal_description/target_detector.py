import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
class YoloTargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        
        # Kamera dinleyici ve gimbal/hedef için yayıncılar
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/camera/process_image', 10)
        self.target_pub = self.create_publisher(Point, '/target/pixel_position', 10)
        
        self.bridge = CvBridge()
        
        # YENİ EĞİTTİĞİNİZ MODELİ BURADA YÜKLÜYORUZ
        pkg_share = get_package_share_directory('gimbal_description')
        model_path = os.path.join(pkg_share, 'weights', 'best.pt')
        self.model = YOLO(model_path)
        self.get_logger().info("YOLOv8 Özel Drone Modeli Başarıyla Yüklendi!")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        # YOLO ile görüntü üzerinde tahmin yap (conf=0.25 varsayılan yeterlidir)
        # verbose=False terminalin YOLO yazılarıyla dolmasını engeller
        results = self.model(cv_image, verbose=False)

        # Eğer model bir nesne tespit ettiyse
        if len(results) > 0 and len(results[0].boxes) > 0:
            # En yüksek güven oranına sahip ilk kutuyu al (Drone)
            box = results[0].boxes[0]
            
            # Kutunun koordinatlarını al (xmin, ymin, xmax, ymax)
            xyxy = box.xyxy[0].cpu().numpy()
            xmin, ymin, xmax, ymax = map(int, xyxy)
            
            # Merkezin piksel konumunu hesapla
            cx = int((xmin + xmax) / 2)
            cy = int((ymin + ymax) / 2)

            # Ekran Ortası Artı Göstergesi (Crosshair)
            cv2.drawMarker(cv_image, (320, 240), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

            # Yapay zekanın bulduğu drone etrafına MAVİ kutu ve MERKEZ noktası çiz
            cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)
            cv2.circle(cv_image, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(cv_image, f"Drone: {box.conf[0]:.2f}", (xmin, ymin - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
           
            # Gimbal takip sistemine yeni merkez koordinatını gönder
            point_msg = Point()
            point_msg.x = float(cx)
            point_msg.y = float(cy)
            self.target_pub.publish(point_msg)

        # İşlenmiş görüntüyü (kutulu halini) ROS 2 üzerinden yayınla
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloTargetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
