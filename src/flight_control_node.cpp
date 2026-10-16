#include <spikive_control_sdk/config.h>
#include <spikive_control_sdk/control_sdk.h>
#include <spikive_control_sdk/localization_sdk.h>
#include <spikive_control_sdk/gotoAction.h>
#include <spikive_control_sdk/landAction.h>
#include <spikive_control_sdk/return_homeAction.h>
#include <spikive_control_sdk/takeoffAction.h>

#include <actionlib/server/action_server.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <set>
#include <string>

namespace spikive_control_sdk
{
namespace
{
enum ErrorCode : std::uint8_t
{
  OK = 0,
  ERR_CANCELLED = 3,
  ERR_POSE_INVALID = 4,
  ERR_NOT_TAKEOFF = 6,
  ERR_ALREADY_FLYING = 7,
  ERR_PRECHECK_FAILED = 12,
  ERR_PARAM_INVALID = 19,
  ERR_NOT_SUPPORTED = 20,
  ERR_BUSY = 21
};

enum class Task { NONE, TAKEOFF, LAND, GOTO, RETURN_HOME };
enum TakeoffPhase : std::uint8_t { PRECHECK = 1, CLIMBING = 2, DONE = 4 };
// 现有状态没有独立的触地检测阶段，保留 Action 定义但不生成 phase=2。
enum LandPhase : std::uint8_t { DESCENDING = 1, TOUCHDOWN = 3, DISARMED = 4 };
// 后端没有独立避障状态，不生成 AVOIDING=2。
enum GotoState : std::uint8_t { NAVIGATING = 1, HOLDING = 3, GOTO_DONE = 4 };
// SDK 补充阶段；没有独立稳定信号，不增加阶段 3。
enum ReturnPhase : std::uint8_t { RETURN_PRECHECK = 1, RETURNING = 2, RETURN_DONE = 4 };

template<class Result>
Result makeResult(bool success, ErrorCode code, const std::string& message)
{
  Result result;
  result.success = success;
  result.error_code = code;
  result.message = message;
  return result;
}
}  // namespace

// Action 适配器：检查请求 → 转发一次命令 → 监视状态并回传结果。
// 互斥覆盖本节点四个飞行 Action，不接管外部控制或目标点发布者。
class FlightControlNode
{
  using TakeoffServer = actionlib::ActionServer<takeoffAction>;
  using LandServer = actionlib::ActionServer<landAction>;
  using GotoServer = actionlib::ActionServer<gotoAction>;
  using ReturnServer = actionlib::ActionServer<return_homeAction>;
  using TakeoffGoal = TakeoffServer::GoalHandle;
  using LandGoal = LandServer::GoalHandle;
  using GotoGoal = GotoServer::GoalHandle;
  using ReturnGoal = ReturnServer::GoalHandle;

public:
  FlightControlNode(ros::NodeHandle& nh, const Config& config)
    : config_(config), sdk_(nh, config),
      takeoff_server_(nh, config.takeoff_action,
                      [this](TakeoffGoal goal) { onTakeoff(goal); },
                      [this](TakeoffGoal goal) { cancelTakeoff(goal); }, false),
      land_server_(nh, config.land_action,
                   [this](LandGoal goal) { onLand(goal); },
                   [this](LandGoal goal) { cancelLand(goal); }, false),
      goto_server_(nh, config.goto_action,
                   [this](GotoGoal goal) { onGoto(goal); },
                   [this](GotoGoal goal) { cancelGoto(goal); }, false),
      return_server_(nh, config.return_home_action,
                     [this](ReturnGoal goal) { onReturn(goal); },
                     [this](ReturnGoal goal) { cancelReturn(goal); }, false)
  {
    const auto takeoff_name = nh.resolveName(config.takeoff_action);
    const auto land_name = nh.resolveName(config.land_action);
    const auto goto_name = nh.resolveName(config.goto_action);
    const auto return_name = nh.resolveName(config.return_home_action);
    if (std::set<std::string>{takeoff_name, land_name, goto_name, return_name,
                              nh.resolveName(config.relocalize_action)}.size() != 5)
    {
      throw std::invalid_argument("Flight actions resolve to the same ROS name");
    }
    sdk_.setUpdateCallback([this]() { onStateUpdate(); });
    feedback_timer_ = nh.createWallTimer(ros::WallDuration(1.0 / config.feedback_rate),
                                        &FlightControlNode::onTimer, this);
    // 沿用 WayPoint 的顺序：Continue 后等待 0.2 s，再发送真实目标。
    // 使用单次定时器避免阻塞状态/取消回调；这不是任务超时。
    goto_dispatch_timer_ = nh.createWallTimer(ros::WallDuration(0.2),
                                             &FlightControlNode::dispatchGoto, this, true, false);
    takeoff_server_.start();
    land_server_.start();
    goto_server_.start();
    return_server_.start();
    ROS_INFO("Flight actions ready for drone %d: %s, %s, %s, %s; speed and timeout remain reserved",
             config.drone_id, config.takeoff_action.c_str(), config.land_action.c_str(),
             config.goto_action.c_str(), config.return_home_action.c_str());
  }

private:
  bool ready() const
  {
    const auto landed = sdk_.extendedState().landed_state;
    return sdk_.hasDroneState() && sdk_.connected() && sdk_.hasControlSubscriber() &&
           sdk_.hasExtendedState() &&
           landed >= mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND &&
           landed <= mavros_msgs::ExtendedState::LANDED_STATE_LANDING &&
           ros::Time::isValid();
  }

  // 上锁状态独立报告；降落结果不等待上锁，也不发出上锁指令。
  bool disarmedAfter(const ros::Time& stamp) const
  {
    const auto& state = sdk_.mavrosState();
    return state.connected && state.header.stamp > stamp && !state.armed;
  }

  landResult makeLandResult(bool success, ErrorCode code, const std::string& message,
                            const ros::Time& stamp) const
  {
    auto result = makeResult<landResult>(success, code, message);
    result.disarmed = disarmedAfter(stamp);
    return result;
  }

  void rejectTakeoff(TakeoffGoal goal, ErrorCode code, const std::string& message)
  {
    goal.setRejected(makeResult<takeoffResult>(false, code, message), message);
    ROS_WARN_STREAM("Takeoff rejected: " << message);
  }

  void rejectLand(LandGoal goal, ErrorCode code, const std::string& message)
  {
    goal.setRejected(makeLandResult(false, code, message, ros::Time(0)), message);
    ROS_WARN_STREAM("Land rejected: " << message);
  }

  void rejectGoto(GotoGoal goal, ErrorCode code, const std::string& message)
  {
    auto result = makeResult<gotoResult>(false, code, message);
    result.final_distance = std::numeric_limits<float>::quiet_NaN();
    goal.setRejected(result, message);
    ROS_WARN_STREAM("Goto rejected: " << message);
  }

  void rejectReturn(ReturnGoal goal, ErrorCode code, const std::string& message)
  {
    goal.setRejected(makeResult<return_homeResult>(false, code, message), message);
    ROS_WARN_STREAM("Return rejected: " << message);
  }

  void onTakeoff(TakeoffGoal goal)
  {
    const auto request = goal.getGoal();
    ROS_INFO("Takeoff request: target_altitude=%g climb_speed=%g timeout=%g; all three fields "
             "are reserved and ignored, px4ctrl retains its configured height and speed",
             request->target_altitude, request->climb_speed, request->timeout);
    if (active_task_ != Task::NONE)
    {
      rejectTakeoff(goal, ERR_BUSY, "A flight action is already active");
      return;
    }
    if (!ready())
    {
      rejectTakeoff(goal, ERR_PRECHECK_FAILED,
                    "Require controller state, connected MAVROS, known extended state, a control subscriber and valid ROS time");
      return;
    }
    // px4ctrl 在 TAKEOFF/LANDING 时也可能报告 landed=true，需由 MAVROS 确认在地面。
    if (sdk_.droneState().tookoff || !sdk_.droneState().landed ||
        sdk_.extendedState().landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {
      rejectTakeoff(goal, ERR_ALREADY_FLYING, "Controller or MAVROS reports that the aircraft is airborne or transitioning");
      return;
    }
    if (!sdk_.hasOdometry())
    {
      rejectTakeoff(goal, ERR_POSE_INVALID, "Require MAVROS odometry with a finite altitude");
      return;
    }

    takeoff_goal_ = goal;
    origin_z_ = sdk_.altitudeZ();
    takeoff_feedback_ = takeoffFeedback();
    takeoff_feedback_.phase = PRECHECK;
    takeoff_goal_.setAccepted("Tracking existing px4ctrl takeoff; request parameters are reserved");
    command_stamp_ = sdk_.takeoff();
    active_task_ = Task::TAKEOFF;
    publishFeedback();
    ROS_INFO("Published cmd=1; waiting for a newer DroneState.tookoff=true");
  }

  void onLand(LandGoal goal)
  {
    const auto request = goal.getGoal();
    ROS_INFO("Land request: descend_speed=%g timeout=%g; both fields are reserved and ignored, "
             "px4ctrl retains its configured landing_speed", request->descend_speed, request->timeout);
    if (active_task_ != Task::NONE)
    {
      rejectLand(goal, ERR_BUSY, "A flight action is already active");
      return;
    }
    if (!ready())
    {
      rejectLand(goal, ERR_PRECHECK_FAILED,
                 "Require controller state, connected MAVROS, known extended state, a control subscriber and valid ROS time");
      return;
    }
    if (sdk_.extendedState().landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {
      rejectLand(goal, ERR_NOT_TAKEOFF, "MAVROS reports that the aircraft is already on the ground");
      return;
    }

    land_goal_ = goal;
    land_feedback_ = landFeedback();
    land_feedback_.phase = DESCENDING;
    land_goal_.setAccepted("Tracking existing px4ctrl landing; request parameters are reserved");
    command_stamp_ = sdk_.land();
    active_task_ = Task::LAND;
    publishFeedback();
    ROS_INFO("Published cmd=2; waiting for newer DroneState.landed and MAVROS ON_GROUND");
  }

  void onReturn(ReturnGoal goal)
  {
    ROS_INFO("Return request: timeout=%g is reserved and ignored; FlightManager retains its home position, "
             "planner retains its speed; no automatic landing", goal.getGoal()->timeout);
    if (active_task_ != Task::NONE)
    {
      rejectReturn(goal, ERR_BUSY, "A flight action is already active");
      return;
    }
    if (!ready() || !sdk_.hasGoalSubscriber())
    {
      rejectReturn(goal, ERR_PRECHECK_FAILED,
                   "Require controller state, connected MAVROS, known extended state, control and goal subscribers, valid ROS time");
      return;
    }
    if (sdk_.extendedState().landed_state != mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
    {
      rejectReturn(goal, ERR_NOT_TAKEOFF, "Return requires MAVROS IN_AIR; takeoff/landing transitions are not ready");
      return;
    }
    if (sdk_.droneState().returned)
    {
      rejectReturn(goal, ERR_PRECHECK_FAILED,
                   "Controller already reports returned; cannot confirm a new return from the existing completion flag");
      return;
    }

    return_goal_ = goal;
    return_pending_stamp_ = ros::Time();
    return_feedback_ = return_homeFeedback();
    return_feedback_.phase = RETURN_PRECHECK;
    return_goal_.setAccepted("Tracking existing return; home and speed remain backend-defined, timeout is reserved");
    command_stamp_ = sdk_.returnHome();
    active_task_ = Task::RETURN_HOME;
    publishFeedback();
    ROS_INFO("Published cmd=3; waiting for post-command returned=false then a newer returned=true; no automatic landing");
  }

  void onGoto(GotoGoal goal)
  {
    const auto request = goal.getGoal();
    if (active_task_ != Task::NONE)
    {
      rejectGoto(goal, ERR_BUSY, "A flight action is already active");
      return;
    }
    for (float value : {request->position[0], request->position[1], request->position[2],
                        request->speed, request->arrival_total_dist, request->arrival_horiz_dist,
                        request->stable_time, request->timeout})
    {
      if (!std::isfinite(value))
      {
        rejectGoto(goal, ERR_PARAM_INVALID, "Goto fields must be finite");
        return;
      }
    }
    if (!ready() || !sdk_.hasGoalSubscriber())
    {
      rejectGoto(goal, ERR_PRECHECK_FAILED,
                 "Require controller state, connected MAVROS, known extended state, control and goal subscribers, valid ROS time");
      return;
    }
    if (sdk_.extendedState().landed_state != mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
    {
      rejectGoto(goal, ERR_NOT_TAKEOFF, "Goto requires MAVROS IN_AIR; takeoff/landing transitions are not ready");
      return;
    }
    if (!sdk_.hasNavigationOdometry())
    {
      rejectGoto(goal, ERR_POSE_INVALID, "Require stamped navigation odometry with finite xyz on " +
                 config_.navigation_odometry_topic + " in frame " + config_.world_frame);
      return;
    }
    // 按项目约定直接使用原生局部 xyz；目标与里程计的原点、坐标轴须由现有系统保持一致。
    const std::array<float, 3> target{{request->position[0], request->position[1], request->position[2]}};
    if (target[2] < -0.1)  // 与 Planner 的原有比较保持一致。
    {
      rejectGoto(goal, ERR_NOT_SUPPORTED, "Existing planner ignores goals with native z below -0.1 m");
      return;
    }
    goto_goal_ = goal;
    goto_target_ = target;
    arrival_total_dist_ = request->arrival_total_dist > 0 ? request->arrival_total_dist : config_.arrival_total_dist;
    arrival_horiz_dist_ = request->arrival_horiz_dist > 0 ? request->arrival_horiz_dist : config_.arrival_horiz_dist;
    stable_time_ = request->stable_time > 0 ? request->stable_time : config_.stable_time;
    goto_sent_ = false;
    stable_since_ = ros::Time();
    last_observation_ = ros::Time();
    goto_feedback_ = gotoFeedback();
    goto_feedback_.state = NAVIGATING;
    ROS_INFO("Goto native xyz=(%g,%g,%g), total<=%g m, horizontal<=%g m, stable=%g s; "
             "speed=%g and timeout=%g are reserved and ignored",
             target[0], target[1], target[2], arrival_total_dist_, arrival_horiz_dist_, stable_time_,
             request->speed, request->timeout);
    goto_goal_.setAccepted("Tracking existing planner goto; speed and timeout are reserved");
    active_task_ = Task::GOTO;
    sdk_.resume();
    goto_dispatch_timer_.stop();
    goto_dispatch_timer_.setPeriod(ros::WallDuration(0.2));
    goto_dispatch_timer_.start();
    publishFeedback();
  }

  void dispatchGoto(const ros::WallTimerEvent&)
  {
    if (active_task_ != Task::GOTO || goto_sent_)
    {
      return;
    }
    if (!ready() || !sdk_.hasGoalSubscriber() || !sdk_.hasNavigationOdometry() ||
        sdk_.extendedState().landed_state != mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
    {
      goto_goal_.setAborted(makeGotoResult(false, ERR_PRECHECK_FAILED,
                                          "Goto conditions changed before goal dispatch; Continue was already forwarded"));
      active_task_ = Task::NONE;
      return;
    }
    command_stamp_ = sdk_.goTo(goto_target_[0], goto_target_[1], goto_target_[2]);
    goto_sent_ = true;
    ROS_INFO("Published GoalSet; waiting for newer reached and odometry satisfying arrival thresholds");
  }

  void onStateUpdate()
  {
    if (active_task_ == Task::TAKEOFF)
    {
      updateTakeoff();
    }
    else if (active_task_ == Task::LAND)
    {
      updateLand();
    }
    else if (active_task_ == Task::GOTO)
    {
      updateGoto();
    }
    else if (active_task_ == Task::RETURN_HOME)
    {
      updateReturn();
    }
  }

  void updateTakeoff()
  {
    const auto& state = sdk_.droneState();
    if (!sdk_.connected() || state.header.stamp <= command_stamp_)
    {
      return;
    }
    if (state.tookoff)
    {
      takeoff_feedback_.phase = DONE;
      takeoff_feedback_.progress = 1.0f;
      publishFeedback();
      takeoff_goal_.setSucceeded(makeResult<takeoffResult>(true, OK, "Controller reports takeoff complete"));
      active_task_ = Task::NONE;
      ROS_INFO("Takeoff succeeded from DroneState.tookoff");
      return;
    }
    // MAVROS 补充控制器丢失的 TAKEOFF 过渡态；完成仍只使用 tookoff。
    const auto& extended = sdk_.extendedState();
    const bool climbing = !state.landed ||
      (extended.header.stamp > command_stamp_ &&
       (extended.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF ||
        extended.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR));
    takeoff_feedback_.phase = climbing ? CLIMBING : PRECHECK;
  }

  void updateLand()
  {
    const auto& state = sdk_.droneState();
    const auto& extended = sdk_.extendedState();
    if (!sdk_.connected() || !sdk_.hasExtendedState() || extended.header.stamp <= command_stamp_)
    {
      return;
    }
    // 原控制器把 LANDING/UNKNOWN 也映射为 landed=true，必须补查真实 ON_GROUND。
    if (state.header.stamp > command_stamp_ && state.landed &&
        extended.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {
      auto result = makeLandResult(true, OK, "Controller and MAVROS confirm touchdown", command_stamp_);
      land_feedback_.phase = result.disarmed ? DISARMED : TOUCHDOWN;
      publishFeedback();
      land_goal_.setSucceeded(result);
      active_task_ = Task::NONE;
      ROS_INFO("Land succeeded from confirmed touchdown; disarmed=%s", result.disarmed ? "true" : "false");
    }
    else if (extended.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR ||
             extended.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_LANDING)
    {
      // LANDING 只说明正在降落，不能据此生成触地检测或着地完成状态。
      land_feedback_.phase = DESCENDING;
    }
  }

  void updateReturn()
  {
    const auto& state = sdk_.droneState();
    if (state.header.stamp <= command_stamp_)
    {
      return;
    }
    // px4ctrl 可能保留 is_returned；新时间戳的 true 也可能沿用旧完成状态。
    // 先观察命令之后的 false，再接受时间更晚的 true；这仍不是命令 ACK。
    if (!state.returned)
    {
      return_pending_stamp_ = std::max(return_pending_stamp_, state.header.stamp);
      return_feedback_.phase = RETURNING;
      return;
    }
    const auto& extended = sdk_.extendedState();
    if (return_pending_stamp_.isZero() || state.header.stamp <= return_pending_stamp_ ||
        !sdk_.connected() || state.landed || extended.header.stamp <= command_stamp_ ||
        extended.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
    {
      return;
    }
    return_feedback_.phase = RETURN_DONE;
    return_feedback_.progress = 1.0f;
    publishFeedback();
    return_goal_.setSucceeded(makeResult<return_homeResult>(true, OK,
      "Controller reports return complete; no landing command was sent"));
    active_task_ = Task::NONE;
    ROS_INFO("Return succeeded from DroneState.returned; landing remains a separate action");
  }

  void refreshGotoFeedback()
  {
    const auto unknown = std::numeric_limits<float>::quiet_NaN();
    goto_feedback_.distance_total = unknown;
    goto_feedback_.distance_horizontal = unknown;
    const auto& odom = sdk_.navigationOdometry();
    if (sdk_.hasNavigationOdometry())
    {
      const double dx = goto_target_[0] - odom.pose.pose.position.x;
      const double dy = goto_target_[1] - odom.pose.pose.position.y;
      const double dz = goto_target_[2] - odom.pose.pose.position.z;
      goto_feedback_.distance_horizontal = static_cast<float>(std::hypot(dx, dy));
      goto_feedback_.distance_total = static_cast<float>(std::hypot(std::hypot(dx, dy), dz));
    }
    // fusion_repub 只填 pose，其默认零 twist 不能作为速度证据；模长仍取 MAVROS。
    const auto& v = sdk_.odometry().twist.twist.linear;
    goto_feedback_.current_speed = sdk_.hasOdometry() &&
      std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z)
      ? static_cast<float>(std::hypot(std::hypot(v.x, v.y), v.z)) : unknown;
  }

  gotoResult makeGotoResult(bool success, ErrorCode code, const std::string& message)
  {
    refreshGotoFeedback();
    auto result = makeResult<gotoResult>(success, code, message);
    result.final_distance = goto_feedback_.distance_total;
    return result;
  }

  void updateGoto()
  {
    if (!goto_sent_)
    {
      return;
    }
    refreshGotoFeedback();
    const auto& state = sdk_.droneState();
    const auto& odom = sdk_.navigationOdometry();
    const bool arrived = sdk_.connected() && sdk_.hasNavigationOdometry() &&
      sdk_.extendedState().landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR &&
      state.header.stamp > command_stamp_ && odom.header.stamp > command_stamp_ &&
      state.reached && !state.landed &&
      goto_feedback_.distance_total <= arrival_total_dist_ &&
      goto_feedback_.distance_horizontal <= arrival_horiz_dist_;
    if (!arrived)
    {
      stable_since_ = ros::Time();
      last_observation_ = ros::Time();
      goto_feedback_.state = NAVIGATING;
      return;
    }
    // 同时要求两个数据源的时间推进，定时反馈不能用缓存累计稳定时长。
    const ros::Time observed = std::min(state.header.stamp, odom.header.stamp);
    if (stable_since_.isZero() || observed < last_observation_)
    {
      stable_since_ = observed;
    }
    last_observation_ = observed;
    goto_feedback_.state = HOLDING;
    if ((observed - stable_since_).toSec() >= stable_time_)
    {
      goto_feedback_.state = GOTO_DONE;
      publishFeedback();
      goto_goal_.setSucceeded(makeGotoResult(true, OK, "Controller reached and goto arrival conditions remained satisfied"));
      active_task_ = Task::NONE;
      ROS_INFO("Goto succeeded; final_distance=%g m", goto_feedback_.distance_total);
    }
  }

  void cancelTakeoff(TakeoffGoal goal)
  {
    if (active_task_ == Task::TAKEOFF && goal == takeoff_goal_)
    {
      const std::string message = "Takeoff tracking cancelled; aircraft control is unchanged";
      takeoff_goal_.setCanceled(makeResult<takeoffResult>(false, ERR_CANCELLED, message), message);
      active_task_ = Task::NONE;
      ROS_INFO_STREAM(message);
    }
  }

  void cancelLand(LandGoal goal)
  {
    if (active_task_ == Task::LAND && goal == land_goal_)
    {
      const std::string message = "Land tracking cancelled; aircraft control is unchanged";
      land_goal_.setCanceled(makeLandResult(false, ERR_CANCELLED, message, command_stamp_), message);
      active_task_ = Task::NONE;
      ROS_INFO_STREAM(message);
    }
  }

  void cancelGoto(GotoGoal goal)
  {
    if (active_task_ == Task::GOTO && goal == goto_goal_)
    {
      goto_dispatch_timer_.stop();
      const std::string message = "Goto tracking cancelled; previously forwarded aircraft commands are unchanged";
      goto_goal_.setCanceled(makeGotoResult(false, ERR_CANCELLED, message), message);
      active_task_ = Task::NONE;
      ROS_INFO_STREAM(message);
    }
  }

  void cancelReturn(ReturnGoal goal)
  {
    if (active_task_ == Task::RETURN_HOME && goal == return_goal_)
    {
      const std::string message = "Return tracking cancelled; aircraft control is unchanged";
      return_goal_.setCanceled(makeResult<return_homeResult>(false, ERR_CANCELLED, message), message);
      active_task_ = Task::NONE;
      ROS_INFO_STREAM(message);
    }
  }

  void publishFeedback()
  {
    if (active_task_ == Task::TAKEOFF)
    {
      takeoff_feedback_.current_altitude = sdk_.hasOdometry()
        ? static_cast<float>(sdk_.altitudeZ() - origin_z_)
        : std::numeric_limits<float>::quiet_NaN();
      takeoff_goal_.publishFeedback(takeoff_feedback_);
    }
    else if (active_task_ == Task::LAND)
    {
      // 高度不可用时保留 NaN，不能把本地坐标 z 当成离地测距，也不能伪造为 0。
      land_feedback_.height = sdk_.groundClearance();
      land_goal_.publishFeedback(land_feedback_);
    }
    else if (active_task_ == Task::GOTO)
    {
      refreshGotoFeedback();
      goto_goal_.publishFeedback(goto_feedback_);
    }
    else if (active_task_ == Task::RETURN_HOME)
    {
      return_goal_.publishFeedback(return_feedback_);
    }
  }

  void onTimer(const ros::WallTimerEvent&) { publishFeedback(); }

  Config config_;
  ControlSdk sdk_;
  TakeoffServer takeoff_server_;
  LandServer land_server_;
  GotoServer goto_server_;
  ReturnServer return_server_;
  ros::WallTimer feedback_timer_;
  ros::WallTimer goto_dispatch_timer_;
  TakeoffGoal takeoff_goal_;
  LandGoal land_goal_;
  GotoGoal goto_goal_;
  ReturnGoal return_goal_;
  takeoffFeedback takeoff_feedback_;
  landFeedback land_feedback_;
  gotoFeedback goto_feedback_;
  return_homeFeedback return_feedback_;
  ros::Time return_pending_stamp_;
  std::array<float, 3> goto_target_{};
  double arrival_total_dist_{0.0};
  double arrival_horiz_dist_{0.0};
  double stable_time_{0.0};
  bool goto_sent_{false};
  ros::Time stable_since_;
  ros::Time last_observation_;
  ros::Time command_stamp_;
  double origin_z_{0.0};
  Task active_task_{Task::NONE};
};
}  // namespace spikive_control_sdk

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spikive_control_sdk_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  try
  {
    const auto config = spikive_control_sdk::Config::load(private_nh);
    spikive_control_sdk::FlightControlNode node(nh, config);
    spikive_control_sdk::LocalizationSdk localization(nh, config);
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("Cannot start flight SDK: " << error.what());
    return 1;
  }
  return 0;
}
