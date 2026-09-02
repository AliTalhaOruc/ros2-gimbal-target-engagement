import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import cv2
import numpy as np
from std_msgs.msg import String
import math
from rclpy.duration import Duration
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(roll/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw

class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')
        # 1. TF Dinleyici
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2. Servis İstemcileri
        self.state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')


        # 3. Mermi Havuzu (Object Pool) Ayarları
        self.pool_size = 30
        self.bullet_pool = [f"bullet_{i}" for i in range(self.pool_size)]
        self.available_bullets = [] # Boştaki mermiler
        self.active_bullets = set()                       # Uçuşta olan mermiler
        self.bullet_status = {}
        self.active_timers = {}
        self.spawn_index = 0

        self.bullet_speed = 25.0
        self.last_shot_time = self.get_clock().now()

        self.bullet_status = {}  # Mermilerin durum sözlüğü

        # 4. Abonelikler ve Yayıncılar
        self.hit_sub = self.create_subscription(
            String,
            '/bullet_hits',
            self.bullet_hit_callback,
            10
        )
        self.sub_target = self.create_subscription(Point, '/target/pixel_position', self.target_callback, 10)
        self.pub_gimbal = self.create_publisher(Float64MultiArray, '/gimbal_controller/commands', 10)

        # 5. Başlangıçta Tüm Havuzu Yarat (Uzakta depola)
        # Gazebo servisinin hazır olmasını bekleyip doğuruyoruz
        self.init_timer = self.create_timer(1.0, self.initialize_bullet_pool_once)

        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
                # 100 Hz yayın için periyot: 1.0 / 100.0 = 0.01 saniye
        self.cmd_timer = self.create_timer(0.01, self.publish_gimbal_command)

        self.pool_initialized = False
        self.prev_target_z = None
        self.prev_target_x = None
        self.prev_target_y = None
        self.prev_tf_time = None

        self.vx_world = 0.0
        self.vy_world = 0.0
        self.vz_world = 0.0
        self.center_x = 320.0
        self.center_y = 240.0
        
        self.kp_yaw = 0.00030
        self.kd_yaw = 0.00005
        self.kp_pitch = 0.00030
        self.kd_pitch = 0.00005
        
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        
        # Ayarlar
        self.turret_z = 0.35
        self.turret_z_offset = 0.15
        self.camera_x_offset = 0.10
        self.barrel_length = 0.00
        
        self.lead_multiplier = 1.5
        self.last_frame_time = None
        self.fps = 30.0  # İlk kare için varsayılan
        self.distance_to_target = 20.0

        self.bullet_id = 0
        
        self.active_timers = {}

        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.002
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 3.0
        self.kalman_initialized = False
        self.get_logger().info('TF Tabanlı Milimetrik Balistik Takip Devrede!')

    def bullet_hit_callback(self, msg):
        """C++ eklentisinden VURMA haberi geldiğinde çalışır."""
        hit_bullet_name = msg.data
        
        if hit_bullet_name in self.active_bullets:
            self.get_logger().info(f"🎯🎯🎯 PYTHON HIT!!! {hit_bullet_name} hedefi başarıyla vurdu!")
            
            # Durumunu HIT yapıyoruz ki recycle_bullet çalıştığında "ISKA" basmasın
            self.bullet_status[hit_bullet_name] = "HIT"

            # 2 saniyelik süreyi beklemeden anında havuza geri dönüştür
            timer_id = f"del_{hit_bullet_name}"
            self.recycle_bullet(hit_bullet_name, timer_id)

    def publish_gimbal_command(self):
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [self.current_yaw, self.current_pitch]
        self.pub_gimbal.publish(cmd_msg)
    def get_dynamic_distance(self):
        try:
            # camera_link ile hedefin odom frame'i arasındaki 3D dönüşümü oku
            # (Eğer TF ağacında 'moving_target/link' görünüyorsa 'odom' yerine onu yazabilirsin)
            t = self.tf_buffer.lookup_transform(
                'odom',
                'moving_target/link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dz = t.transform.translation.z
            
            # 3D Hipotenüs / Öklid Mesafesi
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            return distance

        except Exception as e:
            self.get_logger().warn(f"TF Okuma Hatası: {e}", throttle_duration_sec=2.0)
            return 20.0
        
    def fire_bullet(self):
        if not self.pool_initialized:
            return

        # 1. SERVİS KONTROLÜ: Servis hazır değilse mermiyi harcama, çık!
        if not self.state_client.service_is_ready():
            self.get_logger().warn("SetEntityState servisi henüz hazır değil, bekleniyor...", throttle_duration_sec=2.0)
            return

        now = self.get_clock().now()
        if (now - self.last_shot_time).nanoseconds < 900_000_000: 
            return

        if not self.available_bullets:
            self.get_logger().warn("⚠️ Havuzdaki tüm mermiler kullanımda!", throttle_duration_sec=1.0)
            return

        # 2. TF KONTROLÜ: TF dönüşümü okunamazsa mermiyi havuzdan eksiltmeden çık!
        try:
            t = self.tf_buffer.lookup_transform('world', 'muzzle_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF okunamadı: {ex}', throttle_duration_sec=2.0)
            return

        # Artık hem servis hem TF hazır, mermiyi havuzdan güvenle çekebiliriz
        bullet_name = self.available_bullets.pop(0)
        self.active_bullets.add(bullet_name)
        self.last_shot_time = now

        # Pozisyon ve Dönüş Verileri
        spawn_x = t.transform.translation.x
        spawn_y = t.transform.translation.y
        spawn_z = t.transform.translation.z

        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        r11 = 1 - 2 * (qy**2 + qz**2)
        r21 = 2 * (qx * qy + qz * qw)
        r31 = 2 * (qx * qz - qy * qw)
        # ----------------------------------------------------------------------
        # SIKIŞMAYI ÖNLEYEN OFFSET (15 cm Namlu Önüne Kaydırma)
        # ----------------------------------------------------------------------
        offset_distance = 0.15 # 15 cm namlu önü
        spawn_x += r11 * offset_distance
        spawn_y += r21 * offset_distance
        spawn_z += r31 * offset_distance

        vel_x = self.bullet_speed * r11
        vel_y = self.bullet_speed * r21
        vel_z = self.bullet_speed * r31

        state_msg = EntityState()
        state_msg.name = bullet_name
        state_msg.reference_frame = "world"
        state_msg.pose.position.x = spawn_x
        state_msg.pose.position.y = spawn_y
        state_msg.pose.position.z = spawn_z
        state_msg.pose.orientation.x = qx
        state_msg.pose.orientation.y = qy
        state_msg.pose.orientation.z = qz
        state_msg.pose.orientation.w = qw

        state_msg.twist.linear.x = float(vel_x)
        state_msg.twist.linear.y = float(vel_y)
        state_msg.twist.linear.z = float(vel_z)

        req = SetEntityState.Request()
        req.state = state_msg

        self.state_client.call_async(req)
        self.get_logger().info(f'⚡ [POOL FIRE] Atış yapıldı: {bullet_name}')
        self.bullet_status[bullet_name] = "FLYING"

        timer_id = f"del_{bullet_name}"
        self.active_timers[timer_id] = self.create_timer(
            2.0, 
            lambda b_name=bullet_name, t_id=timer_id: self.recycle_bullet(b_name, t_id)
        )
    def recycle_bullet(self, bullet_name, timer_id):
        """
        2 saniye süre dolduğunda çalışır. 
        Mermi silinmez, ISKA kontrolü yapılır ve havuza geri taşınır.
        """
        # 1. Timer Temizliği
        if timer_id in self.active_timers:
            self.active_timers[timer_id].cancel()
            del self.active_timers[timer_id]

        # 2. ISKA KONTROLÜ
        # Eğer C++ hit callback bu merminin durumunu değiştirmediyse hâlâ FLYING'dir -> ISKA!
        if self.bullet_status.get(bullet_name) == "FLYING":
            self.get_logger().warn(f'❌ ISKA... Mermi hedefe değmeden yok oldu. ({bullet_name})')

        # 3. MERMİYİ UZAKTAKİ DEPOYA TAŞI VE HIZINI SIFIRLA
        if bullet_name in self.active_bullets:
            state_msg = EntityState()
            state_msg.name = bullet_name
            state_msg.pose.position.x = -1000.0
            state_msg.pose.position.y = -1000.0
            state_msg.pose.position.z = -1000.0
            state_msg.twist.linear.x = 0.0
            state_msg.twist.linear.y = 0.0
            state_msg.twist.linear.z = 0.0
            state_msg.reference_frame = "world"

            req = SetEntityState.Request()
            req.state = state_msg
            self.state_client.call_async(req)

            # 4. HAVUZ BELLEK TEMİZLİĞİ
            self.active_bullets.remove(bullet_name)
            self.available_bullets.append(bullet_name)
            self.bullet_status[bullet_name] = "IDLE"
        """Mermiyi yok etmek yerine pasif alana (-1000, -1000, -1000) geri taşır."""
        if timer_id in self.active_timers:
            self.active_timers[timer_id].cancel()
            del self.active_timers[timer_id]

        if bullet_name in self.active_bullets:
            state_msg = EntityState()
            state_msg.name = bullet_name
            state_msg.pose.position.x = -1000.0
            state_msg.pose.position.y = -1000.0
            state_msg.pose.position.z = -1000.0
            state_msg.twist.linear.x = 0.0
            state_msg.twist.linear.y = 0.0
            state_msg.twist.linear.z = 0.0
            state_msg.reference_frame = "world"

            req = SetEntityState.Request()
            req.state = state_msg
            self.state_client.call_async(req)

            self.active_bullets.remove(bullet_name)
            self.available_bullets.append(bullet_name)
            self.bullet_status[bullet_name] = "IDLE"
    def initialize_bullet_pool_once(self):
        """Havuz yüklemesini güvenli zincirleme şekilde başlatır."""
        if self.pool_initialized:
            return

        if not self.spawn_client.service_is_ready():
            self.get_logger().info("Spawn servisi bekleniyor...")
            return

        # Zamanlayıcıyı kapatıyoruz ki sürekli çalışmasın
        self.init_timer.cancel()
        self.get_logger().info("📦 Mermi havuzu Gazebo'ya sırayla yükleniyor...")
        
        # İlk mermiyi doğurma sürecini başlat
        self.spawn_next_bullet()

    def spawn_next_bullet(self):
        """Mermileri Gazebo'yu kilitlemeden zincirleme sırayla oluşturur."""
        if self.spawn_index >= self.pool_size:
            self.pool_initialized = True
            self.get_logger().info(f"✅ Mermi havuzu başarıyla yüklendi! Toplam {len(self.available_bullets)} mermi hazır.")
            return

        b_name = self.bullet_pool[self.spawn_index]
        
        bullet_xml = f"""
        <sdf version="1.6">
          <model name="{b_name}">
            <static>false</static> 
            <link name="link">
              <gravity>false</gravity>
              <inertial>
                <mass>0.05</mass>
                <inertia>
                  <ixx>0.000018</ixx><ixy>0</ixy><ixz>0</ixz>
                  <iyy>0.000018</iyy><iyz>0</iyz>
                  <izz>0.000018</izz>
                </inertia>
              </inertial>
              <visual name="visual">
                <geometry><sphere><radius>0.03</radius></sphere></geometry>
                <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
              </visual>
              <collision name="collision">
                <geometry><sphere><radius>0.03</radius></sphere></geometry>
                <surface>
                  <contact><collide_without_contact>true</collide_without_contact></contact>
                </surface>
              </collision>
            </link>
            <plugin name="bullet_plugin" filename="libBulletPlugin.so"/>
          </model>
        </sdf>
        """

        req = SpawnEntity.Request()
        req.name = b_name
        req.xml = bullet_xml
        req.initial_pose.position.x = -1000.0
        req.initial_pose.position.y = -1000.0
        req.initial_pose.position.z = -1000.0

        # Gazebo yanıt verdiğinde bir sonraki mermiye geç (Callback kullanarak güvenli yükleme)
        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda f, name=b_name: self._on_bullet_spawned(f, name))

    def _on_bullet_spawned(self, future, bullet_name):
        try:
            response = future.result()
            if response.success:
                self.available_bullets.append(bullet_name)
                self.bullet_status[bullet_name] = "IDLE"
            else:
                self.get_logger().error(f"❌ {bullet_name} spawn edilemedi: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"Spawn servisi hatası: {e}")

        # Bir sonraki mermiye geç
        self.spawn_index += 1
        self.spawn_next_bullet()
    def check_and_remove_bullet(self, bullet_name, timer_id):
        from gazebo_msgs.srv import DeleteEntity
        
        # Eğer C++'tan tetiklenen callback bu mermiyi HIT yapmadıysa ISKA'dır
        if self.bullet_status.get(bullet_name) == "FLYING":
            self.get_logger().warn(f'❌ ISKA... Mermi hedefe değmeden yok oldu. ({bullet_name})')
        
        # Mermiyi her halükarda normal adıyla dünyadan sil
        req_del = DeleteEntity.Request()
        req_del.name = bullet_name
        if self.delete_client.service_is_ready():
            self.delete_client.call_async(req_del)

        # Bellek Temizliği
        if timer_id in self.active_timers:
            self.active_timers[timer_id].cancel()
            del self.active_timers[timer_id]
        if bullet_name in self.bullet_status:
            del self.bullet_status[bullet_name]

    def target_callback(self, msg):
        measured_x = np.float32(msg.x)
        measured_y = np.float32(msg.y)

        # 1. FPS HESAPLAMA (Otomatik)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_frame_time is not None:
            dt = now - self.last_frame_time
            if dt > 0:
                instant_fps = 1.0 / dt
                self.fps = (0.9 * self.fps) + (0.1 * instant_fps)
                        # KALMAN DT GÜNCELLE
                #self.kf.transitionMatrix[0, 2] = np.float32(dt)
                #self.kf.transitionMatrix[1, 3] = np.float32(dt)

        self.last_frame_time = now

        self.distance_to_target = self.get_dynamic_distance()

        try:
            tf = self.tf_buffer.lookup_transform(
                "odom",
                "moving_target/link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )

            target_x = tf.transform.translation.x
            target_y = tf.transform.translation.y
            target_z = tf.transform.translation.z  # Z Koordinatı Alındı
            now_tf = self.get_clock().now().nanoseconds * 1e-9

            if self.prev_tf_time is not None:
                dx = target_x - self.prev_target_x
                dy = target_y - self.prev_target_y
                dz = target_z - self.prev_target_z  # Z Değişimi

                # Pozisyon GERÇEKTE değiştiyse (Eşik değerini 1e-6 gibi çok küçük yaptık)
                if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                    dt_tf = now_tf - self.prev_tf_time
                    if dt_tf > 0.001:
                        instant_vx = dx / dt_tf
                        instant_vy = dy / dt_tf
                        instant_vz = dz / dt_tf  # Instant Vz

                        # Hız sıçramasını önlemek için EMA
                        alpha = 0.5
                        self.vx_world = (alpha * instant_vx) + ((1 - alpha) * self.vx_world)
                        self.vy_world = (alpha * instant_vy) + ((1 - alpha) * self.vy_world)
                        self.vz_world = (alpha * instant_vz) + ((1 - alpha) * self.vz_world)

                        # Yalnızca YENİ VERİ geldiğinde zamanı ve konumu güncelle
                        self.prev_target_x = target_x
                        self.prev_target_y = target_y
                        self.prev_target_z = target_z
                        self.prev_tf_time = now_tf

            else:
                # İLK ÇALIŞMA ANI (Sistemin kilitlenmesini engeller)
                self.prev_target_x = target_x
                self.prev_target_y = target_y
                self.prev_target_z = target_z
                self.prev_tf_time = now_tf

        except Exception as e:
            self.get_logger().warn(f"TF Error: {str(e)}")
            return
                # 3. BALİSTİK ÖNLEME HESABI
        tof = self.distance_to_target / self.bullet_speed

        lead_tx = target_x + (self.vx_world * tof)
        lead_ty = target_y + (self.vy_world * tof)
        lead_tz = target_z + (self.vz_world * tof)
        target_yaw = math.atan2(lead_ty, lead_tx)
        # Dikey Hedef Açısı (Pitch)
        # NOT: Taret tabanın z yüksekliği varsa (örn: 0.2m) buraya çıkarım eklenmelidir: (lead_tz - turret_z)
        distance_xy = math.sqrt(lead_tx**2 + lead_ty**2)
        delta_z = lead_tz - self.turret_z
        base_target_pitch = -math.atan2(delta_z, distance_xy)
        target_pitch = base_target_pitch
        image_error_y = measured_y - self.center_y
        camera_pitch_trim = 0.0

        
        #lead_frames = tof * self.fps

        measurement = np.array([[measured_x], [measured_y]], np.float32)

        if not self.kalman_initialized:
            self.kf.statePre = np.array(
                [[measured_x],
                [measured_y],
                [0.0],
                [0.0]],
                np.float32
            )

            self.kf.statePost = self.kf.statePre.copy()

            self.kalman_initialized = True

            self.get_logger().info("Kalman initialized")
            return
        
        # 1. DOĞRU KALMAN SIRASI
       
        
        prediction = self.kf.predict()
        self.kf.correct(measurement)

        # 2. HIZI SINIRLA
        max_vel_px = 25.0
        clean_x = self.kf.statePost[0][0]
        clean_y = self.kf.statePost[1][0]
        vel_x_px = np.clip(prediction[2][0], -max_vel_px, max_vel_px)
        vel_y_px = np.clip(prediction[3][0], -max_vel_px, max_vel_px)


        raw_error_x = self.center_x - measured_x
        raw_error_y = self.center_y - measured_y
        dist_to_center = np.sqrt(raw_error_x**2 + raw_error_y**2)

        yaw_error = target_yaw - self.current_yaw
        while yaw_error > math.pi:
            yaw_error -= 2.0 * math.pi

        while yaw_error < -math.pi:
            yaw_error += 2.0 * math.pi

        # Pitch Hatası
        pitch_error = target_pitch - self.current_pitch

        while pitch_error > math.pi:
            pitch_error -= 2.0 * math.pi
        while pitch_error < -math.pi:
            pitch_error += 2.0 * math.pi

        aim_x = measured_x
        aim_y = measured_y
     
        error_x = self.center_x - aim_x
        error_y = self.center_y - aim_y

        d_error_x = error_x - self.prev_error_x
        d_error_y = error_y - self.prev_error_y
        '''
        total_error = np.sqrt(error_x**2 + error_y**2)
        if total_error < 15.0:
            self.fire_bullet()
        '''
        yaw_ready = abs(yaw_error) < 0.03
        pitch_ready = (abs(pitch_error) < 0.03) and (abs(self.center_y - measured_y) < 15)

       
        if yaw_ready and pitch_ready:
                self.fire_bullet()
               
        # 4. GİMBAL HAREKET HIZINI SINIRLA
        delta_yaw = np.clip(
            1.5 * yaw_error,
            -0.20,
            0.20
        )  

        delta_pitch = np.clip(
            0.5 * pitch_error,
            -0.03,
            0.03
        )
        self.current_yaw += delta_yaw
        self.current_pitch += delta_pitch
        '''
        self.get_logger().info(
        f"my={measured_y:.1f} "
        f"base={base_target_pitch:.3f} "
        f"target={target_pitch:.3f} "
        f"current={self.current_pitch:.3f}"
    )
        '''
        self.current_yaw = max(min(self.current_yaw, 2.5), -2.5)
        self.current_pitch = max(min(self.current_pitch, 0.6), -0.6)



        self.prev_error_x = error_x
        self.prev_error_y = error_y
def main(args=None):
    rclpy.init(args=args)
    node = TargetTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
