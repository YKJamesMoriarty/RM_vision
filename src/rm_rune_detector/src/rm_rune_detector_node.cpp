/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rm_rune_detector_node.cpp
  * @brief      能量机关检测模块
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-12-11      Penguin         1. done
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */

// ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_rune_detector/rune.hpp"
#include "rm_rune_detector/rm_rune_detector_node.hpp"
#include "rm_rune_detector/rune_detector.hpp"

namespace rm_rune_detector
{
    /**
     * @brief RMRuneDetectorNode 的构造函数。
     * @param options 节点选项。
     */
    RMRuneDetectorNode::RMRuneDetectorNode(const rclcpp::NodeOptions &options)
        : Node("rm_rune_detector", options)
    {
        is_detect_rune_ = true;
        RCLCPP_INFO(this->get_logger(), "Start RMRuneDetectorNode!");

        detector_ = InitDetector();

        // Debug Publishers
        debug_ = this->declare_parameter("debug", false);
        if (debug_)
        {
            CreateDebugPublishers();
        }

        // Debug param change moniter
        // debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        // debug_cb_handle_ =
        //     debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter &p)
        //                                              {
        //             debug_ = p.as_bool();
        //             debug_ ? CreateDebugPublishers() : DestroyDebugPublishers(); });

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
            {
                cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
                cam_info_sub_.reset();
            });

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&RMRuneDetectorNode::ImageCallback, this, std::placeholders::_1));

        R_sign_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/rune_detector/R_sign", 10);
        // Visualization Marker Publisher
        // See http://wiki.ros.org/rviz/DisplayTypes/Marker
        R_sign_marker_.ns = "R_sign";
        R_sign_marker_.action = visualization_msgs::msg::Marker::ADD;
        R_sign_marker_.type = visualization_msgs::msg::Marker::CUBE;
        R_sign_marker_.scale.x = 0.1;
        R_sign_marker_.scale.z = 0.1;
        R_sign_marker_.color.a = 1.0;
        R_sign_marker_.color.r = 0.0;
        R_sign_marker_.color.g = 0.5;
        R_sign_marker_.color.b = 1.0;
        R_sign_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
    }

    std::unique_ptr<RuneDetector> RMRuneDetectorNode::InitDetector()
    {
        // Init Rune Detector
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.integer_range.resize(1);
        param_desc.integer_range[0].step = 1;
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 255;
        int binary_thres = declare_parameter("binary_thres", 160, param_desc);

        param_desc.description = "0-RED, 1-BLUE";
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 1;
        auto detect_color = declare_parameter("detect_color", RED, param_desc);

        RuneDetector::HSVParams hsv = {
            .red_min = {3, 0, 100},
            .red_max = {35, 255, 255},
            .blue_min = {77, 0, 199},
            .blue_max = {103, 120, 255},
        };

        RuneDetector::TargetParams t_params = {
            .min_ratio = 0.9,
            .max_ratio = 1.1,
        };

        auto rune_detector = std::make_unique<RuneDetector>(binary_thres, detect_color, t_params, hsv);

        return rune_detector;
    }

    void RMRuneDetectorNode::InitVisualizationMarkerPublishers()
    {
        target_0_pub_ =
            this->create_publisher<visualization_msgs::msg::Marker>("/rune_detector/target_0", 10);
        target_1_pub_ =
            this->create_publisher<visualization_msgs::msg::Marker>("/rune_detector/target_1", 10);
        target_2_pub_ =
            this->create_publisher<visualization_msgs::msg::Marker>("/rune_detector/target_2", 10);
        target_3_pub_ =
            this->create_publisher<visualization_msgs::msg::Marker>("/rune_detector/target_3", 10);
        target_4_pub_ =
            this->create_publisher<visualization_msgs::msg::Marker>("/rune_detector/target_4", 10);
        target_pubs_.push_back(target_0_pub_);
        target_pubs_.push_back(target_1_pub_);
        target_pubs_.push_back(target_2_pub_);
        target_pubs_.push_back(target_3_pub_);
        target_pubs_.push_back(target_4_pub_);

        visualization_msgs::msg::Marker target_marker_0;
        visualization_msgs::msg::Marker target_marker_1;
        visualization_msgs::msg::Marker target_marker_2;
        visualization_msgs::msg::Marker target_marker_3;
        visualization_msgs::msg::Marker target_marker_4;
        target_markers_.push_back(target_marker_0);
        target_markers_.push_back(target_marker_1);
        target_markers_.push_back(target_marker_2);
        target_markers_.push_back(target_marker_3);
        target_markers_.push_back(target_marker_4);

        for (int i = 0; i < 5; i++)
        {
            target_markers_[i].ns = "target";
            target_markers_[i].action = visualization_msgs::msg::Marker::ADD;
            target_markers_[i].type = visualization_msgs::msg::Marker::SPHERE;
            target_markers_[i].scale.x = 1;
            target_markers_[i].scale.y = 1;
            target_markers_[i].scale.z = 0.1;
            target_markers_[i].color.a = 1.0;
            target_markers_[i].color.r = 0.0;
            target_markers_[i].color.g = 1.0;
            target_markers_[i].color.b = 1.0;
            target_markers_[i].lifetime = rclcpp::Duration::from_seconds(0.1);
        }
    }

    /**
     * @brief 处理图像，检测能量机关并预测打击位置的回调函数
     * @param img_msg
     */
    void RMRuneDetectorNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        // TODO: 识别图中的能量机关靶标后发布靶标信息
        auto targets = DetectRunes(img_msg);
    }

    /**
     * @brief 识别图中的能量机关靶标（包括已激活的和未激活的靶标）
     * @param img_msg
     * @return 所有靶标的数组
     */
    std::vector<Target_Image> RMRuneDetectorNode::DetectRunes(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Detecting rune, Debug = %d", debug_);
        // RCLCPP_INFO(this->get_logger(), "Detecting rune, detet color = %s", detector_->detect_color ? "blue" : "red");
        // RCLCPP_INFO(this->get_logger(), "Detecting rune, detet color = %d", detector_->detect_color);
        // RCLCPP_INFO(this->get_logger(), "Detecting rune...");

        // Convert ROS img to cv::Mat
        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

        // Update params
        // detector_->binary_thresh = get_parameter("binary_thres").as_int();
        detector_->detect_color = 1 - get_parameter("detect_color").as_int(); // 这里使用1-是因为serial的数据中设置的是识别装甲板的颜色，也就是对方的颜色，而能量机关则是己方的颜色，所以要取反

        // TODO: 识别图中的能量机关靶标并得到目标列表
        // 识别R标
        R_Sign_Rectangle R_sign_rect = detector_->DetectRSign(img);

        // 根据R标的位置，进行PnP解算，并发布R标的位置
        if (pnp_solver_ != nullptr)
        {
            R_sign_marker_.header = img_msg->header;
            R_sign_array_.markers.clear();
            R_sign_marker_.id = 0;

            cv::Mat rvec, tvec;
            bool success = pnp_solver_->SolvePnP_RSign(R_sign_rect, rvec, tvec);
            if (success)
            {
                R_sign_tvec_ = tvec.clone();
                R_sign_rvec_ = rvec.clone();
                // Fill pose
                R_sign_pose_.x = R_sign_tvec_.at<double>(0); // 先简单赋值一下，后面再加入跟踪器
                R_sign_pose_.y = R_sign_tvec_.at<double>(1);
                R_sign_pose_.z = R_sign_tvec_.at<double>(2);

                R_sign_marker_.pose.position.x = R_sign_pose_.x;
                R_sign_marker_.pose.position.y = R_sign_pose_.y;
                R_sign_marker_.pose.position.z = R_sign_pose_.z;
                // Fill the markers
                R_sign_marker_.id++;
                R_sign_marker_.scale.y = 0.1;
                R_sign_array_.markers.emplace_back(R_sign_marker_);

                cv::Point3f point(R_sign_marker_.pose.position.x,
                                  R_sign_marker_.pose.position.y,
                                  R_sign_marker_.pose.position.z);
                double distance = cv::norm(point);
                RCLCPP_INFO(this->get_logger(), "R sign distance=%f", distance);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "PnP failed!");
            }
            R_sign_array_.markers.emplace_back(R_sign_marker_);
        }
        RMRuneDetectorNode::PublishMarkers();

        // 识别扇叶
        std::vector<Target_Image> targets;
        targets = detector_->DetectTargets(R_sign_pose_, R_sign_tvec_,
                                 cam_info_->k, cam_info_->d);

        for (size_t i = 0; i < targets.size(); i++)
        {
            //计算靶标中心与旋转中心连线的倾角
            double angle = atan2(targets[i].center.y - R_sign_pose_.y, targets[i].center.x - R_sign_pose_.x);
            double dx = rotation_radius_ * cos(angle);
            double dy = rotation_radius_ * sin(angle);

            std::cout<<dx<<" "<<dy<<std::endl;
            
            //填充发布信息
            target_markers_[i].header = img_msg->header;
            target_markers_[i].id = 0;
            target_markers_[i].pose.position.x = R_sign_pose_.x + dx;
            target_markers_[i].pose.position.y = R_sign_pose_.y + dy;
            target_markers_[i].pose.position.z = R_sign_pose_.z;
            // target_pubs_[i]->publish(target_markers_[i]);
        }


        // Publish debug images
        if (debug_)
        {
            binary_img_for_R_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img_for_R).toImageMsg());
            binary_img_for_targets_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img_for_targets).toImageMsg());
            result_img_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "rgb8", detector_->result_img).toImageMsg());
        }

        return targets;
    }

    void RMRuneDetectorNode::PublishMarkers()
    {
        using Marker = visualization_msgs::msg::Marker;
        R_sign_marker_.action = Marker::ADD;
        R_sign_array_.markers.emplace_back(R_sign_marker_);
        R_sign_pub_->publish(R_sign_array_);
    }

    /**
     * @brief 发布调试信息
     */
    void RMRuneDetectorNode::CreateDebugPublishers()
    {
        // lights_data_pub_ =
        //     this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
        // armors_data_pub_ =
        //     this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

        binary_img_for_R_pub_ = image_transport::create_publisher(this, "/rune_detector/binary_img_for_R");
        binary_img_for_targets_pub_ = image_transport::create_publisher(this, "/rune_detector/binary_img_for_targets");
        result_img_pub_ = image_transport::create_publisher(this, "/rune_detector/result_img");
    }

    void RMRuneDetectorNode::DestroyDebugPublishers()
    {
        // lights_data_pub_.reset();
        // armors_data_pub_.reset();

        binary_img_for_R_pub_.shutdown();
        result_img_pub_.shutdown();
    }

    RMRuneDetectorNode::~RMRuneDetectorNode()
    {
        RCLCPP_INFO(this->get_logger(), "RuneDetectorNode destroyed!");
    }
} // namespace rm_rune_detector

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_rune_detector::RMRuneDetectorNode)
