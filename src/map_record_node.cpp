#include <spikive_control_sdk/RecordAction.h>
#include <spikive_control_sdk/action_support.h>
#include <spikive_control_sdk/record_io.h>
#include <actionlib/server/action_server.h>
#include <boost/filesystem.hpp>
#include <rosbag/recorder.h>
#include <std_msgs/String.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <future>
#include <memory>
#include <spawn.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace spikive_control_sdk
{
// 使用 rosbag record 的原生 Recorder。独立子进程使 SIGINT 只结束这次录制，
// 不影响 SDK/SLAM/Planner；posix_spawn 直接传 argv，不执行 shell。
int recordWorker(int argc, char** argv)
{
  if (argc < 7) return 1;
  const auto parent = static_cast<pid_t>(std::stol(argv[2]));
  if (::prctl(PR_SET_PDEATHSIG, SIGINT) != 0 || ::getppid() != parent) return 1;
  ros::M_string remappings;
  remappings["begin_write"] = argv[4];
  ros::init(remappings, "spikive_bag_" + std::to_string(::getpid()));
  rosbag::RecorderOptions options;
  options.prefix = argv[3];
  options.append_date = false;
  options.publish = true;
  options.max_size = std::stoull(argv[5]);
  options.split = options.max_size > 0;
  options.topics.assign(argv + 6, argv + argc);
  return rosbag::Recorder(options).run();
}

class MapRecordNode
{
  using Server = actionlib::ActionServer<RecordAction>;
  using Handle = Server::GoalHandle;
public:
  MapRecordNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
  {
    std::string action;
    pnh.param<std::string>("record/action", action, "/nav/map/record");
    if (action.empty()) throw std::invalid_argument("Record action must not be empty");
    bool split_enabled;
    pnh.param("record/split_enabled", split_enabled, false);
    if (split_enabled)
    {
      int split_size_mib;
      pnh.param("record/split_size_mib", split_size_mib, 500);
      if (split_size_mib <= 0) throw std::invalid_argument("record/split_size_mib must be positive");
      split_size_bytes_ = static_cast<std::uint64_t>(split_size_mib) * 1024 * 1024;
    }
    if (pnh.hasParam("record/topics") && !pnh.getParam("record/topics", default_topics_))
      throw std::invalid_argument("record/topics must be a list of strings");
    if (!default_topics_.empty()) validateRecordingTopics(default_topics_);
    server_.reset(new Server(nh, action, [this](Handle h) { onGoal(h); },
                            [this](Handle h) { onCancel(h); }, false));
    timer_ = nh.createWallTimer(ros::WallDuration(0.05), &MapRecordNode::onTimer, this);
    server_->start();
    ROS_INFO_STREAM("SDK rosbag recorder: " << action << "; split: "
                    << (split_size_bytes_ > 0 ? "enabled" : "disabled"));
  }

  ~MapRecordNode()
  {
    // 正常关闭时保存自身录制；从不向外部 rosbag/SLAM 节点发信号。
    active_ = false;
    const auto until = ros::SteadyTime::now() + ros::WallDuration(5);
    if (pid_ > 0)
    {
      stopping_ = true;
      ::kill(pid_, SIGINT);
      while (pid_ > 0 && ros::SteadyTime::now() < until)
      { pollChild(); if (pid_ > 0) ros::WallDuration(0.05).sleep(); }
      if (pid_ > 0)
      {
        ::kill(pid_, SIGKILL);
        int status;
        while (::waitpid(pid_, &status, 0) < 0 && errno == EINTR) {}
        ROS_ERROR_STREAM("Recorder did not close on shutdown; partial data retained: " << staging_);
      }
    }
    while (merge_.valid() && ros::SteadyTime::now() < until)
    {
      pollMerge();
      if (merge_.valid()) ros::WallDuration(0.05).sleep();
    }
    if (merge_.valid())
    {
      progress_.cancelled = true;
      merge_.wait();
      pollMerge();
    }
  }

private:
  void reject(Handle h, ErrorCode code, const std::string& text)
  { h.setRejected(makeResult<RecordResult>(false, code, text), text); }

  void startWorker(const std::vector<std::string>& topics)
  {
    // mkdtemp 创建私有目录，避免录制期间覆盖既有文件或另一进程的输出。
    std::string pattern = (boost::filesystem::path(destination_).parent_path() / ".spikive-record-XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end()); buffer.push_back('\0');
    if (!::mkdtemp(buffer.data())) throw std::runtime_error("Cannot create recording directory: " + std::string(std::strerror(errno)));
    staging_ = buffer.data();
    const bool split = split_size_bytes_ > 0;
    const auto prefix = (boost::filesystem::path(staging_) /
                         (split ? boost::filesystem::path(destination_).stem() : "record")).string();
    first_part_ = prefix + (split ? "_0.bag" : ".bag");
    progress_.completed = 0;
    progress_.total = 0;
    progress_.cancelled = false;
    const auto ready_topic = pnh_.resolveName("begin_write_" + std::to_string(++sequence_));
    ready_sub_ = nh_.subscribe(ready_topic, 1, &MapRecordNode::onReady, this);
    std::vector<std::string> args = {"spikive_map_record_node", "--record-worker", std::to_string(::getpid()),
                                    prefix, ready_topic, std::to_string(split_size_bytes_)};
    args.insert(args.end(), topics.begin(), topics.end());
    std::vector<char*> argv;
    for (auto& arg : args) argv.push_back(&arg[0]);
    argv.push_back(nullptr);
    // /proc/self/exe 在 devel/install/可重定位安装包中均指向当前 SDK 可执行文件。
    const int error = ::posix_spawn(&pid_, "/proc/self/exe", nullptr, nullptr, argv.data(), environ);
    if (error != 0)
    {
      pid_ = -1;
      ready_sub_.shutdown();
      boost::filesystem::remove(staging_);
      throw std::runtime_error("Cannot start rosbag recorder: " + std::string(std::strerror(error)));
    }
    ready_ = stopping_ = false;
    last_error_.clear();
    ROS_INFO_STREAM("Recording PID " << pid_ << " -> " << destination_);
    for (const auto& topic : topics) ROS_INFO_STREAM("  record topic: " << topic);
  }

  void onGoal(Handle h)
  {
    pollChild();
    pollMerge();
    const auto req = h.getGoal();
    if (active_ || merge_.valid()) { reject(h, ERR_BUSY, "A record command or merge is active"); return; }
    if ((req->cmd != 1 && req->cmd != 2) || !TaskDeadline::valid(req->timeout))
    { reject(h, ERR_PARAM_INVALID, "cmd must be 1/2 and timeout finite/nonnegative"); return; }
    if (req->cmd == 1)
    {
      const auto& topics = req->topics.empty() ? default_topics_ : req->topics;
      if (pid_ > 0) { reject(h, ERR_BUSY, "SDK recording is already active; stop with cmd=2 first"); return; }
      if (!ros::Time::isValid()) { reject(h, ERR_PRECHECK_FAILED, "Require valid ROS time"); return; }
      try
      {
        validateRecordingTopics(topics);
        destination_ = recordingDestination(req->bag_path, req->bag_name);
      }
      catch (const std::exception& e) { reject(h, ERR_PARAM_INVALID, e.what()); return; }
      try { startWorker(topics); }
      catch (const std::exception& e) { reject(h, ERR_INTERNAL, e.what()); return; }
    }
    else
    {
      if (!req->topics.empty() || !req->bag_path.empty() || !req->bag_name.empty())
      { reject(h, ERR_PARAM_INVALID, "cmd=2 uses the current session; leave topics/bag_path/bag_name empty"); return; }
      if (pid_ <= 0)
      {
        reject(h, last_error_.empty() ? ERR_PRECHECK_FAILED : ERR_INTERNAL,
               last_error_.empty() ? "No SDK recording to stop" : last_error_);
        return;
      }
      // 启动命令已取消/超时时也允许停止。先等子进程安装 SIGINT 处理器并打开文件。
      stopping_ = true;
      if (ready_) ::kill(pid_, SIGINT);
    }
    goal_ = h;
    command_ = req->cmd;
    active_ = true;
    deadline_.start(req->timeout);
    goal_.setAccepted(req->cmd == 1 ? "Starting SDK rosbag recording" : "Closing SDK rosbag recording");
    publishFeedback();
  }

  void onReady(const std_msgs::String::ConstPtr& msg)
  {
    // 原生 Recorder 发布的是本次暂存文件名；旧会话回调不能确认新任务。
    if (pid_ <= 0 || msg->data != first_part_ || ready_) return;
    pollChild();
    if (pid_ <= 0) return;
    ready_ = true;
    if (stopping_) ::kill(pid_, SIGINT);
    expire();
    if (active_ && command_ == 1) finish(true, OK, "Recording started; cmd=2 saves " + destination_);
  }

  void pollChild()
  {
    if (pid_ <= 0) return;
    int status = 0;
    const auto waited = ::waitpid(pid_, &status, WNOHANG);
    if (waited == 0 || (waited < 0 && errno == EINTR)) return;
    pid_ = -1;
    ready_sub_.shutdown();
    last_error_.clear();
    if (waited < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
      last_error_ = "Recorder exited abnormally; data retained: " + staging_;
    else
    {
      try
      {
        if (split_size_bytes_ > 0)
        {
          merge_ = std::async(std::launch::async, [this] {
            mergeRecordedParts(staging_, destination_, progress_);
          });
          return;
        }
        publishRecordedBag(first_part_, destination_);
        boost::system::error_code ignored;
        boost::filesystem::remove(first_part_, ignored);
        boost::filesystem::remove(staging_, ignored);
        reportSaved();
      }
      catch (const std::exception& e) { last_error_ = std::string(e.what()) + "; data retained: " + staging_; }
    }
    finishSave();
  }

  void pollMerge()
  {
    if (!merge_.valid() || merge_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    if (active_ && command_ == 2) publishFeedback();
    try
    {
      merge_.get();
      reportSaved();
    }
    catch (const std::exception& e) { last_error_ = std::string(e.what()) + "; data retained: " + staging_; }
    finishSave();
  }

  void reportSaved()
  {
    ROS_INFO_STREAM("Bag saved: " << destination_);
    if (!stopping_) last_error_ = "Recorder exited without an SDK stop; bag saved: " + destination_;
  }

  void finishSave()
  {
    if (!last_error_.empty()) ROS_ERROR_STREAM(last_error_);
    expire();
    if (active_) finish(last_error_.empty(), last_error_.empty() ? OK : ERR_INTERNAL,
                        last_error_.empty() ? "Bag saved: " + destination_ : last_error_);
  }

  void publishFeedback()
  {
    RecordFeedback feedback;
    feedback.state = merge_.valid() && command_ == 2 ? 2 : (ready_ ? 1 : 0);
    feedback.completed_parts = progress_.completed.load();
    feedback.total_parts = progress_.total.load();
    goal_.publishFeedback(feedback);
  }

  void finish(bool success, ErrorCode code, const std::string& text)
  {
    RecordFeedback feedback;
    feedback.state = success ? (command_ == 1 ? 1 : 3) : 4;
    feedback.completed_parts = progress_.completed.load();
    feedback.total_parts = progress_.total.load();
    goal_.publishFeedback(feedback);
    const auto result = makeResult<RecordResult>(success, code, text);
    if (success) goal_.setSucceeded(result); else goal_.setAborted(result);
    active_ = false;
  }

  void expire()
  {
    if (active_ && deadline_.expired())
      finish(false, ERR_TIMEOUT, "Record command timed out; use cmd=2 to stop a running recording; a stop already requested continues");
  }

  void onCancel(Handle h)
  {
    if (!active_ || h != goal_) return;
    goal_.setCanceled(makeResult<RecordResult>(false, ERR_CANCELLED,
      "Tracking cancelled; use cmd=2 to stop a running recording; a stop already requested continues"));
    active_ = false;
  }

  void onTimer(const ros::WallTimerEvent&)
  {
    expire();
    pollChild();
    pollMerge();
    if (active_) publishFeedback();
  }

  ros::NodeHandle nh_, pnh_;
  pid_t pid_{-1};
  bool ready_{false}, stopping_{false}, active_{false};
  unsigned command_{0};
  std::uint64_t sequence_{0};
  std::uint64_t split_size_bytes_{0};
  std::vector<std::string> default_topics_;
  std::string destination_, staging_, first_part_, last_error_;
  RecordingMergeProgress progress_;
  std::future<void> merge_;
  TaskDeadline deadline_;
  ros::Subscriber ready_sub_;
  ros::WallTimer timer_;
  std::unique_ptr<Server> server_;
  Handle goal_;
};
}  // namespace spikive_control_sdk

int main(int argc, char** argv)
{
  try
  {
    if (argc > 1 && std::string(argv[1]) == "--record-worker")
      return spikive_control_sdk::recordWorker(argc, argv);
    ros::init(argc, argv, "spikive_map_record_node");
    ros::NodeHandle nh, pnh("~");
    spikive_control_sdk::MapRecordNode node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& e) { ROS_FATAL_STREAM(e.what()); return 1; }
  return 0;
}
