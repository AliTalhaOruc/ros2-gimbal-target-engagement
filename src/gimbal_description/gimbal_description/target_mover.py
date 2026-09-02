import rclpy
from rclpy.node import Node
import math
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState

class TargetMover(Node):
    def __init__(self):
        super().__init__("target_mover")
        
        # Gazebo Entity State Servis İstemcisi
        self.state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo set_entity_state servisi bekleniyor...')

        self.timer = self.create_timer(0.02, self.move_target)  # 50 Hz (Pürüzsüz akıcı hareket)
        self.t = 0.0
        
        # -----------------------------------------------------------------
        # YÖRÜNGE MODLARI: (Test etmek istediğini buraya yaz reis)
        # "SINUS"             -> Sadece sağa sola akıcı sinüs dalgası çizer
        # "HORIZONTAL_SQUARE" -> Yerde/Yatay düzlemde (X ve Y eksenlerinde) kare çizer
        # "VERTICAL_SQUARE"   -> Havada/Dikey düzlemde (Y ve Z eksenlerinde) kare çizer
        # -----------------------------------------------------------------
        self.mode = "HORIZONTAL_SQUARE"
        
        # Başlangıç Orta Noktası (Ofsetler)
        self.base_x = 4.0
        self.base_y = 0.0
        self.base_z = 0.5  # Drone'un yerden yüksekliği
        
        self.get_logger().info(f"Target Mover Başlatıldı. Aktif Mod: {self.mode}")

    def move_target(self):
        self.t += 0.02
        
        x = self.base_x
        y = self.base_y
        z = self.base_z

        # MOD 1: SAĞA-SOLA SİNÜS HAREKETİ
        if self.mode == "SINUS":
            # X ve Z sabit kalır, Y ekseninde (sağ-sol) 2.5 metre genlikte süzülür
            y = self.base_y + 2.5 * math.sin(self.t * 1.2)

        
                # MOD 2: YERDE YATAY KARE (X ve Y Eksenlerinde) - JET HIZI (Hız: 3.0, Kenar: 5.0)
        elif self.mode == "HORIZONTAL_SQUARE":
            side_length = 5.0  # Karenin bir kenar uzunluğu
            speed = 3.0        # Hareket hızı (m/s) - 2 Katına Çıkarıldı!
            
            # Bir kenarın kat edilme süresi = 5.0 / 3.0 = 1.666667 saniye
            side_duration = side_length / speed 
            total_duration = side_duration * 4.0  # Toplam tam tur süresi (6.6666 saniye)
            
            cycle = (self.t) % total_duration
            half_side = side_length / 2.0  # Ofset hesabı için (2.5 metre)
            
            if cycle < side_duration:       # 1. Kenar: Sağ (Y artar)
                x = self.base_x - half_side
                y = self.base_y - half_side + (cycle / side_duration) * side_length
            elif cycle < (side_duration * 2.0):     # 2. Kenar: İleri (X artar)
                x = self.base_x - half_side + ((cycle - side_duration) / side_duration) * side_length
                y = self.base_y + half_side
            elif cycle < (side_duration * 3.0):     # 3. Kenar: Sol (Y azalır)
                x = self.base_x + half_side
                y = self.base_y + half_side - ((cycle - (side_duration * 2.0)) / side_duration) * side_length
            else:                                   # 4. Kenar: Geri (X azalır)
                x = self.base_x + half_side - ((cycle - (side_duration * 3.0)) / side_duration) * side_length
                y = self.base_y - half_side

        # MOD 3: HAVADA DİKEY KARE (Y ve Z Eksenlerinde)
        elif self.mode == "VERTICAL_SQUARE":
            side_length = 2.0  # Karenin bir kenar uzunluğu
            speed = 1.0        # Hareket hızı
            cycle = (self.t * speed) % 8.0  # 8 saniyede bir tam tur
            
            if cycle < 2.0:       # Sağ (Y artar)
                y = self.base_y - 1.0 + (cycle / 2.0) * side_length
                z = self.base_z - 1.0
            elif cycle < 4.0:     # Yukarı (Z artar)
                y = self.base_y + 1.0
                z = self.base_z - 1.0 + ((cycle - 2.0) / 2.0) * side_length
            elif cycle < 6.0:     # Sol (Y azalır)
                y = self.base_y + 1.0 - ((cycle - 4.0) / 2.0) * side_length
                z = self.base_z + 1.0
            else:                 # Aşağı (Z azalır)
                y = self.base_y - 1.0
                z = self.base_z + 1.0 - ((cycle - 6.0) / 2.0) * side_length

        # Gazebo Mesajının Hazırlanması
        state = EntityState()
        state.name = "moving_target"
        state.reference_frame = "world"
        
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)

        # Servisi Çağır
        req = SetEntityState.Request()
        req.state = state
        self.state_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = TargetMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

