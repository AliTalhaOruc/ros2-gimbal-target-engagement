#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>

namespace gazebo
{
  class BulletPlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;
    private: physics::ModelPtr target_model;
    private: event::ConnectionPtr updateConnection;
    private: double vx, vy, vz;
    private: bool hit_registered = false;
    
    // ROS 2 Yayıncısı ekliyoruz
    private: gazebo_ros::Node::SharedPtr ros_node;
    private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hit_pub;
    private: bool last_active = false;
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->world = _model->GetWorld();

      this->vx = _sdf->HasElement("vx") ? _sdf->Get<double>("vx") : 0.0;
      this->vy = _sdf->HasElement("vy") ? _sdf->Get<double>("vy") : 0.0;
      this->vz = _sdf->HasElement("vz") ? _sdf->Get<double>("vz") : 0.0;

      this->target_model = this->world->ModelByName("moving_target");

      std::string unique_node_name =
      "bullet_plugin_" + this->model->GetName();

      this->ros_node = gazebo_ros::Node::Get(
      _sdf,
      unique_node_name
	);

      this->hit_pub =
      this->ros_node->create_publisher<std_msgs::msg::String>(
        "/bullet_hits", 10);
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&BulletPlugin::OnUpdate, this));
          
          // ==================== AŞAĞIDAKİ LOGU EKLE ====================
      ignition::math::Pose3d spawn_pose = this->model->WorldPose();
      RCLCPP_INFO(this->ros_node->get_logger(),
        "🔴 [GZ SPAWN] %s -> Pos:(%.3f, %.3f, %.3f) Yaw: %.3f",
        this->model->GetName().c_str(),
        spawn_pose.Pos().X(), spawn_pose.Pos().Y(), spawn_pose.Pos().Z(),
        spawn_pose.Rot().Yaw());
      // ============================================================
    }

   public: void OnUpdate()
    {
      // --------------------------------------------------------------------------
      // EKLENEN KISIM 2: Merminin Sahneye Çıkışını Algılama ve Resetleme
      // --------------------------------------------------------------------------
      ignition::math::Vector3d bullet_pos = this->model->WorldPose().Pos();
      
      // Mermi depoda (-1000'de) değilse aktif demektir
      bool active = bullet_pos.Z() > -900.0;

      // Depodan sahneye YENİ fırlatıldıysa (Pasiften Aktife geçtiyse):
      if (active && !last_active)
      {
        this->hit_registered = false; // Hit durumunu sıfırla, tekrar vurabilir!
        
        RCLCPP_INFO(
          this->ros_node->get_logger(),
          "🔄 [RESET HIT] %s tekrar sahneye çıktı, vurmaya hazır!",
          this->model->GetName().c_str()
        );
      }
      
      // Bir sonraki kare için durumu güncelle
      this->last_active = active;

      // --------------------------------------------------------------------------
      // MEVCUT HİT KONTROLÜ
      // --------------------------------------------------------------------------
      // Eğer mermi depodaysa, hedefe önceden çarptıysa veya hedef yoksa işlem yapma
      if (!active || hit_registered || !this->target_model)
        return;

      ignition::math::Vector3d target_pos = this->target_model->WorldPose().Pos();
      double distance = bullet_pos.Distance(target_pos);

      if (distance < 0.30)
      {
        // ROS 2 üzerinden Python'a merminin adını gönderiyoruz
        auto message = std_msgs::msg::String();
        message.data = this->model->GetName();
        this->hit_pub->publish(message);
        
        this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
        this->hit_registered = true;
      }
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(BulletPlugin)
}

