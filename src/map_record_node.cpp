#include <spikive_control_sdk/RecordAction.h>
#include <spikive_control_sdk/action_support.h>
#include <spikive_control_sdk/record_io.h>
#include <actionlib/server/action_server.h>
#include <std_msgs/String.h>
#include <future>
#include <memory>
#include <unistd.h>

namespace spikive_control_sdk
{
// SLAM save_result 的命令/状态适配。保存路径只影响 SDK 导出，不写 SLAM 参数。
class MapRecordNode
{
  using Server = actionlib::ActionServer<RecordAction>;
  using Handle = Server::GoalHandle;
public:
  MapRecordNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param("drone_id", drone_id_, 1);
    if (drone_id_ < 0 || drone_id_ > 32767) throw std::invalid_argument("drone_id out of range");
    const std::string prefix = "/drone_" + std::to_string(drone_id_) + "_bag_record_";
    std::string start, stop, status, action;
    pnh.param<std::string>("record/start_topic", start, "");
    pnh.param<std::string>("record/stop_topic", stop, "");
    pnh.param<std::string>("record/status_topic", status, "");
    pnh.param<std::string>("record/action", action, "/nav/map/record");
    if (start.empty()) start = prefix + "start";
    if (stop.empty()) stop = prefix + "stop";
    if (status.empty()) status = prefix + "status";
    if (action.empty() || nh.resolveName(start) == nh.resolveName(stop) ||
        nh.resolveName(start) == nh.resolveName(status) || nh.resolveName(stop) == nh.resolveName(status))
      throw std::invalid_argument("Recording topics must be distinct and action nonempty");
    start_pub_ = nh.advertise<std_msgs::String>(start, 10, false);
    stop_pub_ = nh.advertise<std_msgs::String>(stop, 10, false);
    status_sub_ = nh.subscribe(status, 20, &MapRecordNode::onStatus, this);
    server_.reset(new Server(nh, action, [this](Handle h) { onGoal(h); },
                            [this](Handle h) { onCancel(h); }, false));
    timer_ = nh.createWallTimer(ros::WallDuration(0.05), &MapRecordNode::onTimer, this);
    server_->start();
    ROS_INFO_STREAM("Map recording adapter: " << action << " -> " << start << " / " << stop << "; " << status);
  }
  ~MapRecordNode()
  {
    if (cancel_export_) *cancel_export_ = true;
    // future 析构等待文件线程结束；不自动停止后端采集。
  }
private:
  void reject(Handle h, ErrorCode code, const std::string& text)
  { h.setRejected(makeResult<RecordResult>(false, code, text), text); }

  void onGoal(Handle h)
  {
    const auto req = h.getGoal();
    if (active_ || export_.valid()) { reject(h, ERR_BUSY, "A record command or file export is active"); return; }
    if ((req->cmd != 1 && req->cmd != 2) || !TaskDeadline::valid(req->timeout))
    { reject(h, ERR_PARAM_INVALID, "cmd must be 1/2 and timeout finite/nonnegative"); return; }
    const auto& publisher = req->cmd == 1 ? start_pub_ : stop_pub_;
    if (!ros::Time::isValid() || !publisher.getNumSubscribers() || !status_sub_.getNumPublishers())
    { reject(h, ERR_PRECHECK_FAILED, "Require SLAM recorder subscriber/status publisher and valid ROS time"); return; }
    if (req->cmd == 1 && (!known_ || recording_ || awaiting_ack_))
    { reject(h, recording_ || awaiting_ack_ ? ERR_BUSY : ERR_PRECHECK_FAILED, "Require confirmed idle recorder before start"); return; }
    if (req->cmd == 2 && !recording_ && !awaiting_ack_ && closed_path_.empty())
    { reject(h, ERR_PRECHECK_FAILED, "No active recording or confirmed closed bag to export"); return; }
    std::string target = destination_;
    try
    {
      if (req->cmd == 1 || !req->bag_path.empty() || !req->bag_name.empty())
        target = recordingDestination(req->bag_path, req->bag_name);
      if (target.empty()) throw std::invalid_argument("Provide bag_path and bag_name to recover a recording started outside this SDK");
    }
    catch (const std::exception& e) { reject(h, ERR_PARAM_INVALID, e.what()); return; }
    goal_ = h;
    destination_ = target;
    command_ = req->cmd;
    active_ = true;
    deadline_.start(req->timeout);
    goal_.setAccepted(req->cmd == 1 ? "Starting backend bag recording" : "Stopping backend recording and exporting bag");
    feedback_.state = recording_ ? 1 : 0;
    goal_.publishFeedback(feedback_);
    if (req->cmd == 2 && !recording_ && !closed_path_.empty() && !awaiting_ack_)
    { startExport(); return; }
    request_id_ = "sdk_record_" + std::to_string(::getpid()) + "_" +
                  std::to_string(ros::SteadyTime::now().toNSec()) + "_" + std::to_string(++sequence_);
    if (req->cmd == 1) closed_path_.clear();
    awaiting_ack_ = true;
    std_msgs::String command;
    command.data = recordCommand(request_id_);
    publisher.publish(command);
  }

  void onStatus(const std_msgs::String::ConstPtr& msg)
  {
    RecordStatus status;
    try { status = parseRecordStatus(msg->data, drone_id_); }
    catch (const std::exception& e) { ROS_WARN_STREAM_THROTTLE(5, "Ignored recorder status: " << e.what()); return; }
    known_ = true;
    recording_ = status.state == "recording";
    if (!awaiting_ack_ || status.request_id != request_id_ ||
        status.phase != (command_ == 1 ? "record_start" : "record_stop")) return;
    awaiting_ack_ = false;
    if (!status.ok || (command_ == 1 && !recording_) || (command_ == 2 && recording_))
    {
      if (active_) finish(false, ERR_INTERNAL, status.error);
      return;
    }
    if (command_ == 2) closed_path_ = status.bag_path;
    if (expire() || !active_) return;  // 晚到 ACK 可恢复后端状态，但不恢复已结束的 Action。
    if (command_ == 1)
    {
      feedback_.state = 1;
      finish(true, OK, "Recording started; stop with cmd=2 to save " + destination_);
    }
    else startExport();
  }

  void startExport()
  {
    const auto source = closed_path_, target = destination_;
    cancel_export_ = std::make_shared<std::atomic_bool>(false);
    const auto cancelled = cancel_export_;
    export_ = std::async(std::launch::async, [source, target, cancelled]()
    {
      try { exportRecordedBag(source, target, *cancelled); return std::string(); }
      catch (const std::exception& e) { return std::string(e.what()) + "; source retained: " + source; }
    });
  }

  void finish(bool success, ErrorCode code, const std::string& text)
  {
    feedback_.state = success ? (command_ == 1 ? 1 : 3) : 4;
    goal_.publishFeedback(feedback_);
    const auto result = makeResult<RecordResult>(success, code, text);
    if (success) goal_.setSucceeded(result); else goal_.setAborted(result);
    active_ = false;
  }

  bool expire()
  {
    if (!active_ || !deadline_.expired()) return false;
    if (cancel_export_) *cancel_export_ = true;
    finish(false, ERR_TIMEOUT, "Record command deadline exceeded; backend recording is unchanged; cmd=2 stops it explicitly");
    return true;
  }

  void onCancel(Handle h)
  {
    if (!active_ || h != goal_) return;
    if (cancel_export_) *cancel_export_ = true;
    goal_.setCanceled(makeResult<RecordResult>(false, ERR_CANCELLED,
      "Tracking/export cancelled; backend recording is unchanged; use cmd=2 to stop"));
    active_ = false;
  }

  void onTimer(const ros::WallTimerEvent&)
  {
    expire();
    if (export_.valid() && export_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      const auto error = export_.get();
      if (active_)
      {
        if (error.empty()) finish(true, OK, "Bag saved: " + destination_);
        else finish(false, ERR_INTERNAL, error);
      }
      // 保留 closed_path_，允许失败后 cmd=2 换路径重试导出。
    }
    if (active_) goal_.publishFeedback(feedback_);
  }

  int drone_id_{1};
  bool known_{false}, recording_{false}, active_{false}, awaiting_ack_{false};
  unsigned command_{0};
  std::uint64_t sequence_{0};
  std::string request_id_, destination_, closed_path_;
  TaskDeadline deadline_;
  ros::Publisher start_pub_, stop_pub_;
  ros::Subscriber status_sub_;
  ros::WallTimer timer_;
  std::unique_ptr<Server> server_;
  Handle goal_;
  RecordFeedback feedback_;
  std::shared_ptr<std::atomic_bool> cancel_export_;
  std::future<std::string> export_;
};
}  // namespace spikive_control_sdk

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spikive_map_record_node");
  ros::NodeHandle nh, pnh("~");
  try { spikive_control_sdk::MapRecordNode node(nh, pnh); ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL_STREAM(e.what()); return 1; }
  return 0;
}
