#include <spikive_control_sdk/localization_sdk.h>
#include <spikive_control_sdk/localization_io.h>
#include <spikive_control_sdk/LoadMap.h>
#include <spikive_control_sdk/LocalizationStatus.h>
#include <spikive_control_sdk/relocalizeAction.h>

#include <actionlib/server/action_server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <stdexcept>

namespace spikive_control_sdk
{
class LocalizationSdk::Impl
{
  using Server = actionlib::ActionServer<relocalizeAction>;
  using Goal = Server::GoalHandle;
public:
  Impl(ros::NodeHandle& nh, const Config& config)
    : config_(config), service_nh_(nh), service_spinner_(2, &service_queue_),
      server_(nh, config.relocalize_action,
              [this](Goal goal) { onGoal(goal); }, [this](Goal goal) { onCancel(goal); }, false)
  {
    config_.validate();
    if (nh.resolveName(config.localization_map_topic) == nh.resolveName(config.map_echo_topic) ||
        nh.resolveName(config.match_status_topic) == nh.resolveName(config.localization_status_topic))
      throw std::invalid_argument("Localization input/output topics resolve to the same name");
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config.localization_map_topic, 1, true);
    // 初值不 latch，避免后端重启后自动再次触发匹配。
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(config.initial_pose_topic, 1, false);
    status_pub_ = nh.advertise<LocalizationStatus>(config.localization_status_topic, 1, true);
    backend_status_sub_ = nh.subscribe(config.match_status_topic, 20, &Impl::onStatus, this);
    map_echo_sub_ = nh.subscribe(config.map_echo_topic, 2, &Impl::onMapEcho, this);
    transform_sub_ = nh.subscribe(config.match_transform_topic, 20, &Impl::onTransform, this);
    status_.header.frame_id = "map";
    status_.state = "idle";
    status_.fitness_score = -1;
    status_.message = "Load a PCD map before relocalization";
    service_nh_.setCallbackQueue(&service_queue_);
    load_service_ = service_nh_.advertiseService(config.load_map_service, &Impl::loadMap, this);
    timer_ = nh.createWallTimer(ros::WallDuration(1.0 / config.localization_status_rate), &Impl::onTimer, this);
    server_.start();
    service_spinner_.start();
    ROS_INFO("Localization SDK ready: %s, %s; backend drone=%d; native map/world frames",
             config.load_map_service.c_str(), config.relocalize_action.c_str(), config.drone_id);
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    map_condition_.notify_all();
    service_spinner_.stop();
  }

private:
  bool backendConnected() const
  {
    return backend_status_sub_.getNumPublishers() > 0 && map_echo_sub_.getNumPublishers() > 0;
  }

  bool loadMap(LoadMap::Request& request, LoadMap::Response& response)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (loading_ || active_)
      {
        response.error_code = 21;
        response.message = "A map load or relocalization action is active";
        return true;
      }
      loading_ = true;
    }
    try
    {
      // 文件 I/O 和 PCL 解码只在独立 Service 队列执行；不占用飞行 Action 回调线程。
      auto map = readLocalizationMap(request.map_path, request.map_md5);
      std::unique_lock<std::mutex> lock(mutex_);
      if (!backendConnected() || map_pub_.getNumSubscribers() == 0 || !ros::Time::isValid())
      {
        response.error_code = 12;
        response.message = "Require localization map subscriber, backend status/map publishers and valid ROS time";
      }
      else
      {
        map_pending_ = true;
        echo_received_ = waiting_received_ = false;
        seen_matching_ = false;
        request_stamp_ = ros::Time();
        map_stamp_ = ros::Time::now();
        status_.map_loaded = status_.match_success = status_.transform_valid = false;
        status_.match_transform = geometry_msgs::TransformStamped();
        status_.state = "loading_map";
        status_.fitness_score = -1;
        status_.attempt_count = 0;
        status_.elapsed_time = 0;
        status_.map_path = request.map_path;
        status_.map_md5 = map.md5;
        status_.message = "PCD forwarded; waiting for backend map echo and waiting_initialpose";
        map.cloud.header.stamp = map_stamp_;
        map_pub_.publish(map.cloud);
        const bool confirmed = map_condition_.wait_for(lock, std::chrono::duration<double>(config_.map_ack_timeout),
          [this]() { return stopping_ || (echo_received_ && waiting_received_); });
        response.success = confirmed && !stopping_ && backendConnected();
        response.error_code = response.success ? 0 : 2;
        response.message = response.success ? "Localization backend acknowledged the loaded map; initial pose is still required"
          : "Map was forwarded but backend acknowledgement was not confirmed; backend may already have changed its map";
        status_.map_loaded = response.success;
        status_.state = response.success ? "waiting_initialpose" : "failed";
        status_.message = response.message;
        map_pending_ = false;
      }
      loading_ = false;
    }
    catch (const std::invalid_argument& error) { finishLoadError(response, 19, error.what()); }
    catch (const std::domain_error& error) { finishLoadError(response, 16, error.what()); }
    catch (const std::exception& error) { finishLoadError(response, 99, error.what()); }
    ROS_INFO("LoadMap success=%s error=%u: %s", response.success ? "true" : "false",
             response.error_code, response.message.c_str());
    return true;
  }

  void finishLoadError(LoadMap::Response& response, std::uint8_t code, const std::string& message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    loading_ = false;
    map_pending_ = false;
    response.success = false;
    response.error_code = code;
    response.message = message;
    // 校验失败未发图，保留原地图及其定位状态。
  }

  void onMapEcho(const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_pending_)
    {
      if (cloud->header.stamp > map_stamp_ && cloud->header.frame_id == "map" &&
          static_cast<std::uint64_t>(cloud->width) * cloud->height >= 5)
      {
        echo_received_ = true;
        map_echo_stamp_ = cloud->header.stamp;
        map_condition_.notify_all();
      }
    }
    else if (status_.map_loaded && cloud->header.stamp != map_echo_stamp_)
    {
      invalidateMap("Backend map changed outside this SDK; load the intended map again");
    }
  }

  void invalidateMap(const std::string& message)
  {
    status_.map_loaded = status_.match_success = status_.transform_valid = false;
    seen_matching_ = false;
    status_.state = "idle";
    status_.message = message;
    if (active_) finishAction(false, 15, message);
  }

  void onStatus(const std_msgs::String::ConstPtr& message)
  {
    MatchStatus match;
    try { match = parseMatchStatus(message->data, config_.drone_id); }
    catch (const std::exception& error)
    {
      ROS_WARN_THROTTLE(5, "Ignoring malformed localization status: %s", error.what());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_pending_)
    {
      if (match.state == "waiting_initialpose")
      {
        waiting_received_ = true;
        map_condition_.notify_all();
      }
      return;
    }
    if (!status_.map_loaded) return;
    if (match.state == "idle")
    {
      invalidateMap("Localization backend returned to idle; map must be loaded again");
      return;
    }
    if (request_stamp_.isZero()) return;
    if (!seen_matching_)
    {
      // 忽略旧的 latched localized；每轮必须先观察到初值之后的 matching 起点。
      if (match.state != "matching" || match.attempt_count != 0 || match.fitness_score >= 0) return;
      seen_matching_ = true;
    }
    else if (match.state == "waiting_initialpose")
    {
      invalidateMap("Backend reset to waiting_initialpose during matching");
      return;
    }
    status_.state = match.state;
    status_.message = match.message;
    status_.fitness_score = match.fitness_score;
    status_.threshold = match.threshold;
    status_.attempt_count = match.attempt_count;
    status_.elapsed_time = match.elapsed_time;
    status_.match_success = false;
    if (match.state == "failed")
    {
      if (active_) finishAction(false, 17, match.message);
      return;
    }
    maybeComplete();
  }

  void onTransform(const nav_msgs::Odometry::ConstPtr& odom)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.map_loaded || request_stamp_.isZero() || odom->header.stamp <= request_stamp_ ||
        odom->header.frame_id != "map") return;
    if (status_.transform_valid && odom->header.stamp <= status_.match_transform.header.stamp) return;
    auto pose = odom->pose.pose;
    if (!normalizePose(pose)) return;
    // global_match 的 diff_odom 实际为 T_map_world，原 child_frame_id=body 不能当成机体位姿。
    status_.match_transform.header = odom->header;
    status_.match_transform.child_frame_id = "world";
    status_.match_transform.transform.translation.x = pose.position.x;
    status_.match_transform.transform.translation.y = pose.position.y;
    status_.match_transform.transform.translation.z = pose.position.z;
    status_.match_transform.transform.rotation = pose.orientation;
    status_.transform_valid = true;
    maybeComplete();
  }

  static bool normalizePose(geometry_msgs::Pose& pose)
  {
    const auto& p = pose.position;
    auto& q = pose.orientation;
    for (double value : {p.x, p.y, p.z, q.x, q.y, q.z, q.w}) if (!std::isfinite(value)) return false;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(norm) || norm < 1e-12) return false;
    q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
    return true;
  }

  void onGoal(Goal goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto reject = [&goal](std::uint8_t code, const std::string& message)
    {
      relocalizeResult result;
      result.success = false; result.error_code = code; result.message = message; result.fitness_score = -1;
      goal.setRejected(result, message);
    };
    if (loading_ || active_) { reject(21, "A map load or relocalization action is active"); return; }
    if (!status_.map_loaded) { reject(15, "Load and acknowledge a map through this SDK first"); return; }
    if (!backendConnected() || pose_pub_.getNumSubscribers() == 0 || transform_sub_.getNumPublishers() == 0 ||
        !ros::Time::isValid()) { reject(12, "Localization backend topics or ROS time unavailable"); return; }
    auto pose = goal.getGoal()->initial_pose;
    if (!pose.header.frame_id.empty() && pose.header.frame_id != "map")
    { reject(19, "initial_pose must use the native map frame; no coordinate conversion is performed"); return; }
    if (!normalizePose(pose.pose.pose)) { reject(4, "Initial pose must be finite with a nonzero quaternion"); return; }
    for (double value : pose.pose.covariance)
      if (!std::isfinite(value)) { reject(4, "Initial pose covariance must be finite"); return; }
    goal_ = goal;
    active_ = true;
    seen_matching_ = false;
    request_stamp_ = ros::Time::now();
    status_.tracking = true;
    status_.state = "matching";
    status_.match_success = status_.transform_valid = false;
    status_.match_transform = geometry_msgs::TransformStamped();
    status_.fitness_score = -1;
    status_.attempt_count = 0;
    status_.elapsed_time = 0;
    status_.message = "Initial pose forwarded; waiting for backend matching acknowledgement";
    pose.header.frame_id = "map";
    pose.header.stamp = request_stamp_;
    goal_.setAccepted(status_.message);
    pose_pub_.publish(pose);
    publishFeedback();
  }

  void maybeComplete()
  {
    status_.match_success = seen_matching_ && backendConnected() && status_.map_loaded &&
      status_.state == "localized" && status_.transform_valid && status_.attempt_count > 0 &&
      status_.fitness_score >= 0 && status_.fitness_score < status_.threshold;
    if (active_ && status_.match_success) finishAction(true, 0, status_.message);
  }

  relocalizeResult result(bool success, std::uint8_t code, const std::string& message) const
  {
    relocalizeResult out;
    out.success = success; out.error_code = code; out.message = message;
    out.fitness_score = status_.fitness_score; out.threshold = status_.threshold;
    out.attempt_count = status_.attempt_count;
    out.transform_valid = status_.transform_valid;
    out.match_transform = status_.match_transform;
    return out;
  }

  void finishAction(bool success, std::uint8_t code, const std::string& message)
  {
    publishFeedback();
    if (success) goal_.setSucceeded(result(true, code, message), message);
    else goal_.setAborted(result(false, code, message), message);
    active_ = false;
    status_.tracking = false;
  }

  void onCancel(Goal goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || goal != goal_) return;
    const std::string message = "Relocalization tracking cancelled; backend matching continues";
    goal_.setCanceled(result(false, 3, message), message);
    active_ = false;
    status_.tracking = false;
    status_.message = message;
  }

  void publishFeedback()
  {
    if (!active_) return;
    relocalizeFeedback feedback;
    feedback.state = status_.state;
    feedback.fitness_score = status_.fitness_score;
    feedback.threshold = status_.threshold;
    feedback.attempt_count = status_.attempt_count;
    feedback.elapsed_time = status_.elapsed_time;
    goal_.publishFeedback(feedback);
  }

  void onTimer(const ros::WallTimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.header.stamp = ros::Time::now();
    status_.backend_connected = backendConnected();
    if (!status_.backend_connected) status_.match_success = false;
    status_pub_.publish(status_);
    publishFeedback();
  }

  Config config_;
  ros::CallbackQueue service_queue_;
  ros::NodeHandle service_nh_;
  ros::AsyncSpinner service_spinner_;
  ros::ServiceServer load_service_;
  Server server_;
  Goal goal_;
  ros::Publisher map_pub_, pose_pub_, status_pub_;
  ros::Subscriber backend_status_sub_, map_echo_sub_, transform_sub_;
  ros::WallTimer timer_;
  std::mutex mutex_;
  std::condition_variable map_condition_;
  LocalizationStatus status_;
  bool stopping_{false}, loading_{false}, map_pending_{false};
  bool echo_received_{false}, waiting_received_{false}, active_{false}, seen_matching_{false};
  ros::Time map_stamp_, map_echo_stamp_, request_stamp_;
};

LocalizationSdk::LocalizationSdk(ros::NodeHandle& nh, const Config& config)
  : impl_(new Impl(nh, config)) {}
LocalizationSdk::~LocalizationSdk() = default;
}  // namespace spikive_control_sdk
