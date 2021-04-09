#include <memory>
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include "rmw/qos_profiles.h"
#include "sensor_msgs/image_encodings.hpp"
#include "rclcpp/rclcpp.hpp"

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

using std::placeholders::_1;

class AVisualizerNode : public rclcpp::Node
{
  public:
    AVisualizerNode(std::string _arucoImgTopic) : Node("arucoVisualizer"), arucoImgTopic(_arucoImgTopic)
    {
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

      // Create subscriber
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(arucoImgTopic, qos, std::bind(&AVisualizerNode::GrabImage, this, _1));
    }

  private:
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::string arucoImgTopic;
};

void AVisualizerNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msgRGB)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try
  {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_INFO(this->get_logger(), "B %s", e.what());
    return;
  }

  if(cv_ptrRGB)
  {
    cv::imshow("Detected Aruco", cv_ptrRGB->image);
  }
}

//
// Main
//
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string arucoImgTopic = "/arucoDetector_image";

  auto nodePtr = std::make_shared<AVisualizerNode>(arucoImgTopic);

  rclcpp::spin(nodePtr);
  rclcpp::shutdown();
  return 0;
}
