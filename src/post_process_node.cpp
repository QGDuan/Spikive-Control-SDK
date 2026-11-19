#include <spikive_control_sdk/PostProcessAction.h>
#include <spikive_control_sdk/action_support.h>
#include <spikive_control_sdk/post_process_io.h>
#include <spikive_control_sdk/record_io.h>

#include <actionlib/server/action_server.h>
#include <boost/filesystem.hpp>
#include <ros/master.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace spikive_control_sdk
{
namespace
{
namespace fs = boost::filesystem;

struct PostProcessConfig
{
  std::string action{"/nav/map/post_process"};
  std::string pointcloud_topic;
  std::string input_directory;
  std::string output_directory;
  std::vector<std::string> lio_nodes;
  std::uint64_t max_points{0};
  double feedback_period{0.2};
};

std::string joinNodes(const std::vector<std::string>& nodes)
{
  std::string result;
  for (const auto& node : nodes)
  {
    if (!result.empty()) result += ", ";
    result += node;
  }
  return result;
}
}  // namespace

class PostProcessNode
{
  using Server = actionlib::ActionServer<PostProcessAction>;
  using Handle = Server::GoalHandle;

public:
  PostProcessNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
  {
    int drone_id = 1;
    pnh_.param("drone_id", drone_id, drone_id);

    pnh_.param<std::string>("post_process/action", config_.action, config_.action);
    if (config_.action.empty()) throw std::invalid_argument("post_process/action must not be empty");

    std::string topic;
    pnh_.param<std::string>("post_process/pointcloud_topic", topic, "");
    if (topic.empty()) topic = "/drone_" + std::to_string(drone_id) + "_cloud_registered";
    if (topic.empty() || topic.front() != '/')
      throw std::invalid_argument("post_process/pointcloud_topic must be an absolute ROS topic");
    config_.pointcloud_topic = topic;

    std::string input, output;
    pnh_.param<std::string>("post_process/input_directory", input, "");
    pnh_.param<std::string>("post_process/output_directory", output, "");
    if (input.empty())
      pnh_.param<std::string>("record/output_directory", input, DEFAULT_RECORD_OUTPUT_DIRECTORY);
    if (output.empty()) output = "~/spikive-output";
    config_.input_directory = expandHome(input);
    config_.output_directory = expandHome(output);
    if (config_.input_directory.empty() || config_.input_directory.front() != '/')
      throw std::invalid_argument("post_process/input_directory must be absolute");
    if (config_.output_directory.empty() || config_.output_directory.front() != '/')
      throw std::invalid_argument("post_process/output_directory must be absolute");
    ensureOutputDirectory(config_.output_directory);

    int max_points = 0;
    pnh_.param("post_process/max_points", max_points, max_points);
    if (max_points < 0) throw std::invalid_argument("post_process/max_points must be nonnegative");
    config_.max_points = static_cast<std::uint64_t>(max_points);

    pnh_.param("post_process/feedback_period", config_.feedback_period, config_.feedback_period);
    if (!std::isfinite(config_.feedback_period) || config_.feedback_period <= 0.0)
      throw std::invalid_argument("post_process/feedback_period must be finite and positive");

    if (!pnh_.getParam("post_process/lio_nodes", config_.lio_nodes))
      config_.lio_nodes = {"/laserMapping", "/slam_to_uav_transform"};

    server_.reset(new Server(nh_, config_.action, [this](Handle h) { onGoal(h); },
                             [this](Handle h) { onCancel(h); }, false));
    timer_ = nh_.createWallTimer(ros::WallDuration(0.05), &PostProcessNode::onTimer, this);
    last_feedback_ = ros::SteadyTime::now();
    server_->start();
    ROS_INFO_STREAM("SDK post-process action: " << config_.action << "; cloud topic: "
                    << config_.pointcloud_topic << "; output: " << config_.output_directory);
  }

  ~PostProcessNode()
  {
    cancel_.store(true);
    if (future_.valid())
    {
      future_.wait();
      pollJob();
    }
    if (job_active_)
    {
      job_goal_.setAborted(makeResult<PostProcessResult>(
        false, ERR_INTERNAL, "Post-process node shutting down; partial data may be retained"));
      job_active_ = false;
    }
    if (stop_active_)
    {
      stop_goal_.setAborted(makeResult<PostProcessResult>(
        false, ERR_INTERNAL, "Post-process node shutting down"));
      stop_active_ = false;
    }
  }

private:
  void reject(Handle h, ErrorCode code, const std::string& text)
  {
    h.setRejected(makeResult<PostProcessResult>(false, code, text), text);
  }

  std::string outcomeText(const PostProcessOutcome& outcome) const
  {
    std::string text = outcome.message;
    if (outcome.messages > 0 || outcome.points > 0)
    {
      text += " [messages=" + std::to_string(outcome.messages) +
              ", points=" + std::to_string(outcome.points) + "]";
    }
    if (!outcome.frame_id.empty()) text += " frame=" + outcome.frame_id;
    if (!outcome.partial_path.empty()) text += "; partial PCD: " + outcome.partial_path;
    return text;
  }

  PostProcessResult resultFor(const PostProcessOutcome& outcome) const
  {
    return makeResult<PostProcessResult>(outcome.success, outcome.code, outcomeText(outcome));
  }

  void publishJobFeedback(std::uint8_t state)
  {
    if (!job_active_) return;
    PostProcessFeedback feedback;
    feedback.state = state;
    job_goal_.publishFeedback(feedback);
  }

  void publishStopFeedback(std::uint8_t state)
  {
    if (!stop_active_) return;
    PostProcessFeedback feedback;
    feedback.state = state;
    stop_goal_.publishFeedback(feedback);
  }

  bool lioStillRunning(std::string* names)
  {
    std::vector<std::string> nodes;
    if (!ros::master::getNodes(nodes))
    {
      if (names != nullptr) *names = "cannot query ROS master";
      return true;
    }
    if (!hasForbiddenNode(nodes, config_.lio_nodes))
    {
      if (names != nullptr) names->clear();
      return false;
    }
    std::vector<std::string> running;
    for (const auto& node : nodes)
    {
      std::string normalized = node;
      if (!normalized.empty() && normalized.front() != '/') normalized.insert(normalized.begin(), '/');
      for (const auto& forbidden : config_.lio_nodes)
      {
        std::string expected = forbidden;
        if (!expected.empty() && expected.front() != '/') expected.insert(expected.begin(), '/');
        if (normalized == expected)
        {
          running.push_back(normalized);
          break;
        }
      }
    }
    if (names != nullptr) *names = joinNodes(running);
    return true;
  }

  void onGoal(Handle h)
  {
    pollJob();
    const auto req = h.getGoal();
    if (job_active_ || future_.valid() || stop_active_)
    {
      reject(h, ERR_BUSY, "A post-process command is active");
      return;
    }
    if ((req->cmd != 1 && req->cmd != 2) || !TaskDeadline::valid(req->timeout))
    {
      reject(h, ERR_PARAM_INVALID, "cmd must be 1/2 and timeout finite/nonnegative");
      return;
    }

    if (req->cmd == 1)
    {
      std::string running;
      if (lioStillRunning(&running))
      {
        reject(h, ERR_PRECHECK_FAILED,
               running.empty() ? "Cannot verify LIO state before post-process"
                               : "SLAM/LIO is still running: " + running + "; stop it before post-process");
        return;
      }

      std::string bag_path;
      std::string output_path;
      try
      {
        bag_path = resolveInputBag(req->bag_path, req->bag_name, config_.input_directory);
        const std::string stem = fs::path(bag_path).stem().string();
        output_path = resolveOutputPath(req->map_path, req->map_name, config_.output_directory, stem);
        ensureOutputDirectory(fs::path(output_path).parent_path().string());
      }
      catch (const PostProcessError& error)
      {
        reject(h, error.code(), error.what());
        return;
      }
      catch (const std::exception& error)
      {
        reject(h, ERR_INTERNAL, std::string("Cannot prepare post-process: ") + error.what());
        return;
      }

      PostProcessRunRequest request;
      request.bag_path = bag_path;
      request.output_path = output_path;
      request.options.pointcloud_topic = config_.pointcloud_topic;
      request.options.max_points = config_.max_points;

      job_goal_ = h;
      job_active_ = true;
      cancel_.store(false);
      job_deadline_.start(req->timeout);
      try
      {
        future_ = std::async(std::launch::async, [this, request]() {
          PostProcessControl control;
          control.cancelled = &cancel_;
          control.expired = [this]() { return job_deadline_.expired(); };
          return runPostProcess(request, control);
        });
      }
      catch (const std::exception& error)
      {
        job_active_ = false;
        reject(h, ERR_INTERNAL, std::string("Cannot start post-process: ") + error.what());
        return;
      }
      h.setAccepted("Starting point cloud post-process");
      last_feedback_ = ros::SteadyTime::now();
      publishJobFeedback(1);
      return;
    }

    // cmd=2: stop the current post-process. The worker checks cancel_ and the
    // original goal is finalized after it exits; the stop goal is ACKed then.
    if (!req->bag_path.empty() || !req->bag_name.empty() || !req->map_path.empty() ||
        !req->map_name.empty())
    {
      reject(h, ERR_PARAM_INVALID, "cmd=2 uses the current session; leave bag/map fields empty");
      return;
    }
    if (!job_active_ || !future_.valid())
    {
      reject(h, ERR_PRECHECK_FAILED, "No post-process is running");
      return;
    }
    if (stop_active_)
    {
      reject(h, ERR_BUSY, "A stop request is already active");
      return;
    }
    stop_goal_ = h;
    stop_active_ = true;
    stop_deadline_.start(req->timeout);
    cancel_.store(true);
    h.setAccepted("Stopping point cloud post-process");
    last_feedback_ = ros::SteadyTime::now();
    publishJobFeedback(1);
    publishStopFeedback(1);
  }

  void onCancel(Handle h)
  {
    if (job_active_ && h == job_goal_)
    {
      cancel_.store(true);
      return;
    }
    if (stop_active_ && h == stop_goal_)
    {
      stop_active_ = false;
    }
  }

  void pollJob()
  {
    if (!job_active_ || !future_.valid()) return;
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;

    PostProcessOutcome outcome;
    try
    {
      outcome = future_.get();
    }
    catch (const std::exception& error)
    {
      outcome.success = false;
      outcome.code = ERR_INTERNAL;
      outcome.message = std::string("Post-process worker failed: ") + error.what();
    }

    const auto result = resultFor(outcome);
    if (outcome.success)
    {
      publishJobFeedback(3);
      job_goal_.setSucceeded(result);
      job_active_ = false;
      if (stop_active_)
      {
        publishStopFeedback(3);
        stop_goal_.setSucceeded(makeResult<PostProcessResult>(
          true, OK, "Post-process completed before the stop request"));
        stop_active_ = false;
      }
      return;
    }

    publishJobFeedback(4);
    if (outcome.code == ERR_CANCELLED)
      job_goal_.setCanceled(result);
    else
      job_goal_.setAborted(result);
    job_active_ = false;

    if (stop_active_)
    {
      if (outcome.code == ERR_CANCELLED)
      {
        publishStopFeedback(3);
        stop_goal_.setSucceeded(makeResult<PostProcessResult>(
          true, OK, "Post-process stopped; partial data may be retained"));
      }
      else
      {
        publishStopFeedback(4);
        stop_goal_.setAborted(result);
      }
      stop_active_ = false;
    }
  }

  void onTimer(const ros::WallTimerEvent&)
  {
    pollJob();
    if (!job_active_) return;

    const auto now = ros::SteadyTime::now();
    if ((now - last_feedback_).toSec() >= config_.feedback_period)
    {
      last_feedback_ = now;
      publishJobFeedback(1);
      if (stop_active_) publishStopFeedback(1);
    }
    if (job_deadline_.expired()) cancel_.store(true);
    if (stop_active_ && stop_deadline_.expired())
    {
      publishStopFeedback(4);
      stop_goal_.setAborted(makeResult<PostProcessResult>(
        false, ERR_TIMEOUT, "Stop request timed out; cancellation remains requested"));
      stop_active_ = false;
    }
  }

  ros::NodeHandle nh_, pnh_;
  PostProcessConfig config_;
  std::unique_ptr<Server> server_;
  ros::WallTimer timer_;
  Handle job_goal_, stop_goal_;
  bool job_active_{false};
  bool stop_active_{false};
  std::atomic<bool> cancel_{false};
  TaskDeadline job_deadline_, stop_deadline_;
  std::future<PostProcessOutcome> future_;
  ros::SteadyTime last_feedback_;
};
}  // namespace spikive_control_sdk

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "spikive_post_process_node");
    ros::NodeHandle nh, pnh("~");
    spikive_control_sdk::PostProcessNode node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}
