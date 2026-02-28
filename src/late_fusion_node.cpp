#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <map>
#include <vector>
#include <string>

class FusionNode : public rclcpp::Node
{
public:
  FusionNode() : Node("late_fusion_node")
  {
    // ---------------- Parameters ----------------
    declare_parameter("rate", 10.0);
    declare_parameter("det_inputs", std::vector<std::string>());
    declare_parameter("img_inputs", std::vector<std::string>());
    declare_parameter("output_det", "/fused/detections");
    declare_parameter("output_img", "/fused/debug_image");

    declare_parameter("grid_rows", 2);
    declare_parameter("grid_cols", 3);
    declare_parameter("tile_width", 320);
    declare_parameter("tile_height", 240);

    rate_ = get_parameter("rate").as_double();
    det_topics_ = get_parameter("det_inputs").as_string_array();
    img_topics_ = get_parameter("img_inputs").as_string_array();
    output_det_topic_ = get_parameter("output_det").as_string();
    output_img_topic_ = get_parameter("output_img").as_string();

    grid_rows_ = static_cast<size_t>(get_parameter("grid_rows").as_int());
    grid_cols_ = static_cast<size_t>(get_parameter("grid_cols").as_int());
    tile_width_ = static_cast<size_t>(get_parameter("tile_width").as_int());
    tile_height_ = static_cast<size_t>(get_parameter("tile_height").as_int());
    grid_cells_ = grid_rows_ * grid_cols_;

    if (det_topics_.empty() || img_topics_.empty())
      throw std::runtime_error("Detection and image topics required");

    // ---------------- Startup Info ----------------
    RCLCPP_INFO(
        get_logger(),
        "Fusion node started | Camera Count: %zu | Grid: %zux%zu | Tile: %zux%zu",
        img_topics_.size(),
        grid_rows_,
        grid_cols_,
        tile_width_,
        tile_height_);

    if (img_topics_.size() > grid_cells_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Camera count (%zu) exceeds grid capacity (%zu). Extra cameras will be ignored.",
          img_topics_.size(),
          grid_cells_);
    }

    // ---------------- Detection Subscriptions ----------------
    for (const auto & topic : det_topics_)
    {
      recv_seq_[topic] = 0;
      last_pub_seq_[topic] = 0;
      latest_detections_[topic] = nullptr;

      auto sub = create_subscription<vision_msgs::msg::Detection2DArray>(
          topic, 10,
          [this, topic](vision_msgs::msg::Detection2DArray::SharedPtr msg)
          {
            latest_detections_[topic] = msg;
            recv_seq_[topic]++;
          });

      det_subs_.push_back(sub);
    }

    // ---------------- Image Subscriptions ----------------
    for (const auto & topic : img_topics_)
    {
      latest_images_[topic] = nullptr;

      auto sub = create_subscription<sensor_msgs::msg::Image>(
          topic, 10,
          [this, topic](sensor_msgs::msg::Image::SharedPtr msg)
          {
            latest_images_[topic] = msg;
          });

      img_subs_.push_back(sub);
    }

    // ---------------- Publishers ----------------
    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        output_det_topic_, 10);

    img_pub_ = create_publisher<sensor_msgs::msg::Image>(
        output_img_topic_, 10);

    // ---------------- Timer ----------------
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
        std::bind(&FusionNode::publishFusion, this));
  }

private:

  void publishFusion()
  {
    publishDetections();
    publishImages();
  }

  // ----------------------------------------------------
  // Detection Freshness Logic
  // ----------------------------------------------------
  void publishDetections()
  {
    vision_msgs::msg::Detection2DArray out_msg;
    out_msg.header.stamp = now();
    out_msg.header.frame_id = "fused";

    bool has_fresh = false;

    for (const auto & topic : det_topics_)
    {
      if (recv_seq_[topic] > last_pub_seq_[topic] &&
          latest_detections_[topic] != nullptr)
      {
        has_fresh = true;

        auto msg = latest_detections_[topic];
        for (const auto & det : msg->detections)
          out_msg.detections.push_back(det);
      }
    }

    if (!has_fresh)
    {
      // Publish empty fused detection message
      out_msg.detections.clear();
    }

    det_pub_->publish(out_msg);

    // Snapshot sequence counters
    for (const auto & topic : det_topics_)
      last_pub_seq_[topic] = recv_seq_[topic];
  }

  // ----------------------------------------------------
  // Image Stitching Logic (Publisher-aware)
  // ----------------------------------------------------
  void publishImages()
  {
    std::vector<cv::Mat> tiles;
    tiles.reserve(grid_cells_);

    for (size_t i = 0; i < grid_cells_; ++i)
    {
      if (i < img_topics_.size())
      {
        const auto & topic = img_topics_[i];

        // Check if publisher still exists
        auto pubs = get_publishers_info_by_topic(topic);
        bool publisher_connected = !pubs.empty();

        if (!publisher_connected)
        {
          tiles.push_back(
              cv::Mat::zeros(tile_height_, tile_width_, CV_8UC3));
          continue;
        }

        if (latest_images_[topic] != nullptr)
        {
          try
          {
            cv::Mat img =
                cv_bridge::toCvCopy(latest_images_[topic], "bgr8")->image;

            cv::Mat resized;
            cv::resize(img, resized,
                       cv::Size(static_cast<int>(tile_width_),
                                static_cast<int>(tile_height_)));

            tiles.push_back(resized);
            continue;
          }
          catch (...)
          {
          }
        }
      }

      // Fallback black tile
      tiles.push_back(
          cv::Mat::zeros(tile_height_, tile_width_, CV_8UC3));
    }

    std::vector<cv::Mat> rows;
    rows.reserve(grid_rows_);

    for (size_t r = 0; r < grid_rows_; ++r)
    {
      std::vector<cv::Mat> row_tiles;
      row_tiles.reserve(grid_cols_);

      for (size_t c = 0; c < grid_cols_; ++c)
      {
        size_t idx = r * grid_cols_ + c;
        row_tiles.push_back(tiles[idx]);
      }

      cv::Mat row;
      cv::hconcat(row_tiles, row);
      rows.push_back(row);
    }

    cv::Mat panorama;
    cv::vconcat(rows, panorama);

    auto out_img =
        cv_bridge::CvImage(std_msgs::msg::Header(),
                           "bgr8", panorama)
            .toImageMsg();

    out_img->header.stamp = now();
    out_img->header.frame_id = "fused";

    img_pub_->publish(*out_img);
  }

  // ---------------- Members ----------------
  double rate_;

  size_t grid_rows_;
  size_t grid_cols_;
  size_t tile_width_;
  size_t tile_height_;
  size_t grid_cells_;

  std::vector<std::string> det_topics_;
  std::vector<std::string> img_topics_;
  std::string output_det_topic_;
  std::string output_img_topic_;

  std::vector<rclcpp::Subscription<
      vision_msgs::msg::Detection2DArray>::SharedPtr> det_subs_;

  std::vector<rclcpp::Subscription<
      sensor_msgs::msg::Image>::SharedPtr> img_subs_;

  rclcpp::Publisher<
      vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;

  rclcpp::Publisher<
      sensor_msgs::msg::Image>::SharedPtr img_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string,
           vision_msgs::msg::Detection2DArray::SharedPtr>
      latest_detections_;

  std::map<std::string,
           sensor_msgs::msg::Image::SharedPtr>
      latest_images_;

  std::map<std::string, uint64_t> recv_seq_;
  std::map<std::string, uint64_t> last_pub_seq_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}
