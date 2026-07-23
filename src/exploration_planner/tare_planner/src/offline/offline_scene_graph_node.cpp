/**
 * @file offline_scene_graph_node.cpp
 * @brief Production entry point of the offline scene-graph pipeline: a ROS 2
 *        node that listens for a signal and constructs the scene graph.
 *
 * Protocol (same request/response shape as the demo-mode responder):
 *   - request topic (std_msgs/String, default /scene_graph_generator/request):
 *       * data == trigger_keyword (default "generate")  -> run on the
 *         `session_dir` parameter;
 *       * data == an absolute path to an existing folder -> run on that
 *         session folder (overrides the parameter for this run);
 *       * anything else is ignored (logged at DEBUG).
 *   - response topic (std_msgs/String, default /scene_graph_generator/response):
 *     one JSON message per run --
 *       {"status":"complete","session":...,"output_dir":...,
 *        "scene_graph_path":...,"scene_graph":{...merged multifloor graph...},
 *        "floors":[{"floor","rooms","nav_nodes","nav_edges","nodes_in_rooms"}
 *                  | {"floor","skipped_reason"}]}
 *     or {"status":"error","message":...} / {"status":"busy","message":...}.
 *
 * The pipeline runs on a worker thread so the executor (and this protocol)
 * stays responsive; one run at a time.
 */

#include <atomic>
#include <exception>
#include <filesystem>
#include <string>
#include <thread>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <nlohmann/json.hpp>

#include "offline/offline_pipeline.h"

namespace osg = offline_scene_graph;
using json = nlohmann::json;

class OfflineSceneGraphNode : public rclcpp::Node
{
public:
  OfflineSceneGraphNode() : Node("offline_scene_graph_node")
  {
    session_dir_ = this->declare_parameter<std::string>("session_dir", "");
    config_yaml_ = this->declare_parameter<std::string>("config_yaml", "");
    output_dir_ = this->declare_parameter<std::string>("output_dir", "");
    if (config_yaml_.empty()) {
      // Default to the installed pipeline yaml (config/ installs flattened into
      // the package share dir). Missing => compiled defaults, with a warning.
      try {
        const std::string installed =
            ament_index_cpp::get_package_share_directory("tare_planner") +
            "/offline_scene_graph.yaml";
        if (std::filesystem::exists(installed)) {
          config_yaml_ = installed;
        }
      } catch (const std::exception &) {
        // fall through to compiled defaults
      }
      if (config_yaml_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "no config_yaml and no installed offline_scene_graph.yaml; "
                    "using compiled defaults");
      }
    }
    RCLCPP_INFO(this->get_logger(), "config: %s",
                config_yaml_.empty() ? "(compiled defaults)" : config_yaml_.c_str());
    building_ = this->declare_parameter<std::string>("building", "");
    trigger_keyword_ = this->declare_parameter<std::string>("trigger_keyword", "generate");
    const std::string request_topic =
        this->declare_parameter<std::string>("request_topic", "/scene_graph_generator/request");
    const std::string response_topic =
        this->declare_parameter<std::string>("response_topic", "/scene_graph_generator/response");

    response_pub_ = this->create_publisher<std_msgs::msg::String>(response_topic, 5);
    request_sub_ = this->create_subscription<std_msgs::msg::String>(
        request_topic, 5,
        [this](std_msgs::msg::String::ConstSharedPtr msg) { OnRequest(msg->data); });

    RCLCPP_INFO(this->get_logger(),
                "ready: '%s' (or a session path) on %s -> scene graph + response on %s",
                trigger_keyword_.c_str(), request_topic.c_str(), response_topic.c_str());
  }

  ~OfflineSceneGraphNode() override
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void OnRequest(const std::string &data)
  {
    // Every received request is logged at INFO: "sent the signal but nothing
    // happened" must be diagnosable as transport (no log at all) vs payload
    // (logged but ignored) from the node's console alone.
    RCLCPP_INFO(this->get_logger(), "request received: '%s'", data.c_str());
    std::string session;
    if (data == trigger_keyword_) {
      session = session_dir_;
    } else if (!data.empty() && std::filesystem::is_directory(data)) {
      session = data;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "ignoring request '%s' (expected '%s' or an existing directory path)",
                  data.c_str(), trigger_keyword_.c_str());
      return;
    }
    if (session.empty()) {
      PublishStatus("error", "no session: set the session_dir parameter or send a path");
      return;
    }

    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
      PublishStatus("busy", "a pipeline run is already in progress");
      return;
    }
    if (worker_.joinable()) {
      worker_.join();  // reap the previous (finished) run
    }
    RCLCPP_INFO(this->get_logger(), "starting pipeline on %s", session.c_str());
    worker_ = std::thread([this, session]() { Run(session); });
  }

  void Run(const std::string &session)
  {
    try {
      osg::PipelineConfig cfg;
      cfg.session_dir = session;
      cfg.config_yaml = config_yaml_;
      cfg.output_dir = output_dir_;
      cfg.building = building_;
      const osg::PipelineResult result = osg::RunOfflinePipeline(cfg);

      json floors = json::array();
      for (const osg::FloorResult &fr : result.floors) {
        if (!fr.skipped_reason.empty()) {
          floors.push_back({ { "floor", fr.floor },
                             { "skipped_reason", fr.skipped_reason } });
          continue;
        }
        floors.push_back({ { "floor", fr.floor },
                           { "rooms", fr.rooms },
                           { "nav_nodes", fr.nav_nodes },
                           { "nav_edges", fr.nav_edges },
                           { "nodes_in_rooms", fr.nodes_in_rooms } });
      }
      json response = { { "status", result.AnyCompleted() ? "complete" : "error" },
                        { "session", session },
                        { "output_dir", result.output_dir },
                        { "floors", std::move(floors) } };
      if (result.AnyCompleted()) {
        response["scene_graph_path"] = result.scene_graph_path;
        response["scene_graph"] = result.scene_graph;
      }
      std_msgs::msg::String msg;
      msg.data = response.dump();
      response_pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "pipeline finished for %s", session.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "pipeline failed: %s", e.what());
      PublishStatus("error", e.what());
    }
    running_.store(false);
  }

  void PublishStatus(const std::string &status, const std::string &message)
  {
    std_msgs::msg::String msg;
    msg.data = json{ { "status", status }, { "message", message } }.dump();
    response_pub_->publish(msg);
  }

  std::string session_dir_, config_yaml_, output_dir_, building_, trigger_keyword_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
  std::atomic<bool> running_{ false };
  std::thread worker_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OfflineSceneGraphNode>());
  rclcpp::shutdown();
  return 0;
}
