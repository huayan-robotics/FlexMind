#! /usr/bin/bash

# 默认值：不清理
CLEANUP=false

# 解析命令行参数
while getopts ":c" opt; do
  case $opt in
    c)
      CLEANUP=true
      ;;
    \?)
      echo "无效的选项: -$OPTARG"
      exit 1
      ;;
  esac
done

# 执行清理操作
if [ "$CLEANUP" = true ]; then
  echo "重新编译"
  catkin_make clean
  catkin_make -DPYTHON_EXECUTABLE=~/fleximind-ros1/venv38/bin/python -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
else
  catkin_make -DPYTHON_EXECUTABLE=~/fleximind-ros1/venv38/bin/python -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
fi




