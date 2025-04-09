#include "System.h"
#include "cv_bridge/cv_bridge.h" // IWYU pragma: keep
#include "opencv2/highgui/highgui.hpp"
#include "orb_slam3_wrapper.h"
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rmw/qos_profiles.h>
#include <rmw/types.h>
#include <utility>

#define BEST_EFFORT_QOS rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data)
#define RELIABLE_QOS rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_services_default)

using namespace std;


//OrbSlam3Mono功能
//1,创建并包含orb3对象
//      1,数据结构变化:
//            1)KF和MP新增uuid: 唯一标识KF和MP. 随机生成，基本保证了唯一性(有极低极低概率冲突). 涉及到序列化/反序列化,presave/poseload的mnid标识都替换为uuid
//            2)KF和MP新增creatorAgentId: 标识哪一台Ag创建的
//            3)KeyFrameDatabase新增映射uuidToKeyFrame: 可以从uuid转换到KF*
// 
//2,单独开一个线程，只接收实时图像输入，并交给orb3处理: grab_compressed_image/grab_image >> pSLAM->TrackMonocular
// 
//3,执行多Ag协同交互任务，包括：
//      1,[本身作为客户端]，创建与其他Ag通信的Peer对象，主动向其他Ag发送最新Kf数据等.(在发送线程中处理)(生成新关键帧时触发)
//            1)向未merge的Ag发送查询Kf的检索描述子: 针对每个Agj,单独记录已发送的KF,然后从Map中收集未发送的KF，打包序列化发送过去.(未作主Ag检查)
//            2)向已merge的Ag发送共享Kf和Mp:       针对每个Agj,单独记录已发送的KF和MP,然后从Map中收集未发送的KF和MP,打包序列化发送过去
//      2,[本身作为服务端]，被动注册数据接收监听，处理其它Ag发送过来的数据.(直接在接收线程处理?)
//            1)接收其它Ag发送的Kf的检索描述子: receiveNewKeyFrameBows (要求本Ag是主Ag)(要求对方Ag未merge) 
//                  a)Kf与本机地图描述子检索
//                  b)检索成功的，进一步尝试PNP重定位: 直接在本机上重定位即可. 
//                    (本框架是谁Ag的id小，就参考谁的地图，并把小id的Ag的地图发送给大id的Ag去)
//                    大id的Ag作为接收端得到参考地图并尝试回环Merge：receiveMapToAttemptMerge >> LoopClosing::InsertKeyFrame
//                    本机执行merge成功，会将成功的Agid和对应的sim3变换记录在Atlas::successfullyMergedAgentIds
//                    本机执行merge成功，如果对方Agid比自己小，就会刷新本机的参考Agid(Gpid)和对齐变换，并通知本组其它Ag成员一起刷新:OrbSlam3Wrapper::updateSuccessfullyMerged
//            2)接收其他Ag发送的共享Kf和Mp: receiveNewKeyFrames 
//                  a)将exKF和exMP反序列化出来，并恢复链接关系，但是先不加入本机地图map
//                  b)将exKF交给LM线程处理:
//                    将地图点加入本机map,将本exKF加入本机map
//                    本exKF和本机map的地图点进行融合
//                    本exKF执行LBA
class OrbSlam3Mono : public OrbSlam3Wrapper {
public:
  OrbSlam3Mono()
    : OrbSlam3Wrapper("orb_slam3_mono", ORB_SLAM3::System::MONOCULAR) {

    this->declare_parameter("imageTopic", "robot" + to_string(agentId) + "/camera/image_color");
    string imageTopic = this->get_parameter("imageTopic").as_string();

    this->declare_parameter("reliableImageTransport", true);
    bool reliableImageTransport = this->get_parameter("reliableImageTransport").as_bool();

    this->declare_parameter("compressedImage", true);
    bool compressedImage = this->get_parameter("compressedImage").as_bool();

    image_subscriber_thread = std::thread([this, imageTopic, reliableImageTransport, compressedImage]() {
      auto sub_node = rclcpp::Node::make_shared("image_subscriber_thread_node");
      if (compressedImage) {
        compressed_image_subscriber = sub_node->create_subscription<sensor_msgs::msg::CompressedImage>(imageTopic,
          reliableImageTransport ? RELIABLE_QOS : BEST_EFFORT_QOS,
          std::bind(&OrbSlam3Mono::grab_compressed_image, this, std::placeholders::_1));
      }
      else {
        image_subscriber = sub_node->create_subscription<sensor_msgs::msg::Image>(imageTopic,
          reliableImageTransport ? RELIABLE_QOS : BEST_EFFORT_QOS,
          std::bind(&OrbSlam3Mono::grab_image, this, std::placeholders::_1));
      }
      rclcpp::spin(sub_node);
    });

    run();
  };

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscriber;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread image_subscriber_thread;

  void grab_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // image must be copied since it uses the conversion_mat_ for storage
      // which is asynchronously overwritten in the next callback invocation
      cv::Mat image_cpy = cv_ptr->image.clone();

      rclcpp::Time timestamp = msg->header.stamp;
      process_image(image_cpy, timestamp);

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  void grab_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // image must be copied since it uses the conversion_mat_ for storage
      // which is asynchronously overwritten in the next callback invocation
      cv::Mat image_cpy = cv_ptr->image.clone();

      rclcpp::Time timestamp = msg->header.stamp;
      process_image(image_cpy, timestamp);

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
  }

  //本地slam处理实时图像数据入口
  void process_image(const cv::Mat& image, const rclcpp::Time& timestamp) {
    double seconds = timestamp.nanoseconds() * 1.e-9;
    RCLCPP_INFO(this->get_logger(), "New Image. Timestamp: %f", seconds);
    Sophus::SE3f Tcw = pSLAM->TrackMonocular(image, seconds);

    publishRosVizTopics->publish_camera_pose(Tcw.inverse(), timestamp);

    newFrameProcessed = true;
    lastFrameTimestamp = timestamp;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OrbSlam3Mono>();

  rclcpp::shutdown();
  return 0;
}