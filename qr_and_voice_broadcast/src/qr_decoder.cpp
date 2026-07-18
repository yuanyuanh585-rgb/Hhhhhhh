#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp" 
#include <cv_bridge/cv_bridge.h>               
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "zbar.h"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
// #include <string.h> // string.h is a C header. <string> is C++ but c_str() is part of std::string.
                       // Implicitly included by other headers often.
#include "origincar_msg/msg/sign.hpp" // For the unused sign_pub
#include "ai_msgs/msg/perception_targets.hpp"

class QrCodeDetection : public rclcpp::Node
{
public:
  QrCodeDetection() : Node("image_subscriber")
  {
    // Subscribe to CompressedImage now
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/image2", 10, std::bind(&QrCodeDetection::imageCallback, this, std::placeholders::_1));

    sign_com_pub = this->create_publisher<std_msgs::msg::String>(
      "/number", 10); // Changed QoS to 10, a common default. Original was 1.

    switch_branch_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "route_strategy/switch_branch", 10);

    perception_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        "/origincar_competition", 10, std::bind(&QrCodeDetection::handleAiMsg, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "qrcode node has been started!");

    hasqrcodr = false;
    has_switched_ = false;
  }

private:
  void handleAiMsg(const ai_msgs::msg::PerceptionTargets::SharedPtr msg_ptr)
  {
        for (auto &target : msg_ptr->targets) {
        if (!target.rois.empty()) {
            if(target.type == "qr") 
            {
                // const auto& roi = target.rois[0]; 
                maxRoi = target.rois[0];
                RCLCPP_INFO(this->get_logger(), "PerceptionTargets.x1:%d,y1:%d,width:%d,height:%d",maxRoi.rect.x_offset,maxRoi.rect.y_offset,maxRoi.rect.width,maxRoi.rect.height);
                hasqrcodr = true;
            }
        }
    }
  }
  // Callback now receives CompressedImage
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // RCLCPP_ERROR(this->get_logger(), "decode compressed image111111111.");
    // 是否存在二维码
    if(!hasqrcodr)
      return ;
    // RCLCPP_INFO(this->get_logger(), "decode compressed image.");
    try {
      // Decode the compressed image
      // msg->data is a std::vector<uint8_t>
      cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

      if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image.");
        return;
      }
      
      int x1 = maxRoi.rect.x_offset -30;
      int y1 = maxRoi.rect.y_offset -30;
      int width1 = maxRoi.rect.width + 60;
      int height1 = maxRoi.rect.height + 60;

      x1 = std::max(0, x1);  // 不能小于 0
      y1 = std::max(0, y1);  // 不能小于 0
      width1 = std::min(width1, frame.cols - x1);  // 不能超出图像右边界
      height1 = std::min(height1, frame.rows - y1); // 不能超出图像下边界

      cv::Rect roi(x1,y1,width1,height1);
      // RCLCPP_INFO(this->get_logger(), "decode compressed image.x1:%d,y1:%d,width:%d,height:%d",x1,y1,width1,height1);
      cv::Mat cropped = frame(roi);
      cv::Mat gray;
      cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);
      cv::resize(gray, gray, cv::Size(), 2, 2, cv::INTER_LINEAR);
      // RCLCPP_INFO(this->get_logger(), "decode compressed image.");
      zbar::ImageScanner scanner;
      // 初始化ZBar扫描器
      scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
      // 增加扫描密度提高识别率
      scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_X_DENSITY, 3);
      scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_Y_DENSITY, 3);
      // scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

      // Create ZBar image from the grayscale OpenCV Mat
      zbar::Image zbar_image(gray.cols, gray.rows, "Y800", (uchar *)gray.data, gray.cols * gray.rows);
      
      int n = scanner.scan(zbar_image);
      if (n > 0) {
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); 
            symbol != zbar_image.symbol_end(); ++symbol) {
          const char *qrCode_msg = symbol->get_data().c_str();
          RCLCPP_INFO(this->get_logger(), "Scanned QR Code: %s", qrCode_msg);

          auto sign_com_msg = std_msgs::msg::String();
          sign_com_msg.data = qrCode_msg;

          sign_com_pub->publish(sign_com_msg);
          // sign_com_pub->publish(std::move(sign_com_msg));
          RCLCPP_INFO(this->get_logger(), "Published QR Data: %s", sign_com_msg.data.c_str());

          // 解码成功后按奇偶切换路线（仅触发一次）
          if (!has_switched_) {
            auto switch_msg = std_msgs::msg::Int32();
            try {
              int qr_number = std::stoi(qrCode_msg);
              switch_msg.data = (qr_number % 2 != 0) ? 2 : 3;  // 奇数→route_2 顺时针, 偶数→route_3 逆时针
              RCLCPP_INFO(this->get_logger(),
                "QR=%d (%s), switching to route_%d!",
                qr_number, (qr_number % 2 != 0) ? "odd→clockwise" : "even→counterclockwise", switch_msg.data);
            } catch (const std::exception &) {
              switch_msg.data = 2;  // 解析失败默认 route_2
              RCLCPP_WARN(this->get_logger(),
                "QR '%s' is not a number, default switching to route_2!", qrCode_msg);
            }
            switch_branch_pub_->publish(switch_msg);
            has_switched_ = true;
          }
        }
      } else {
        // Optional: Log if no QR codes are found in a frame
        // RCLCPP_INFO(this->get_logger(), "No QR codes found in this frame.");
      }
      
      zbar_image.set_data(NULL, 0); // Clean up zbar image data buffer
      hasqrcodr = false;
    }
    catch (const cv::Exception &e) { // Catch OpenCV specific errors
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
      hasqrcodr = false;
      return;
    }
    catch (const std::exception &e) { // Catch other standard library errors
      RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
      hasqrcodr = false;
      return;
    }
    // Note: cv_bridge::Exception is less likely here since we are not using toCvCopy for the compressed image.
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_; // Updated type
  // The following publisher 'sign_pub' is declared but not initialized in the constructor or used in the code.
  // If it's not needed, it can be removed.
  rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sign_com_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr switch_branch_pub_;

  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr perception_subscription_;//二维码识别信息

  ai_msgs::msg::Roi maxRoi;
  bool hasqrcodr;
  bool has_switched_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QrCodeDetection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}