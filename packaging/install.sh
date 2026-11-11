#!/usr/bin/env bash
# 安装包根目录使用；只在解压目录构建，不启动 ROS 节点或修改其他工作区。
set -eo pipefail

sdk_release_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sdk_build_jobs="${SDK_BUILD_JOBS:-2}"
if [[ ! "$sdk_build_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo 'SDK_BUILD_JOBS 必须为正整数。' >&2
  exit 1
fi
if [[ ! -f "$sdk_release_root/src/Spikive-Control-SDK/package.xml" ]]; then
  echo '请在完整安装包内运行根目录的 install.sh。' >&2
  exit 1
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo '请先安装 ROS1 Noetic 及 README_安装.md 中的系统依赖。' >&2
  exit 1
fi

# 明确只使用系统 Noetic 作为 underlay，不继承其他 SDK/Planner 工作区。
source /opt/ros/noetic/setup.bash
export CMAKE_PREFIX_PATH=/opt/ros/noetic
cd "$sdk_release_root"
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF \
  -j"$sdk_build_jobs" -l"$sdk_build_jobs"
printf '\nSDK v1.5.0 构建完成。每个使用终端执行：\nsource %q\n' "$sdk_release_root/devel/setup.bash"
