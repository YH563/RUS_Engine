cmake_minimum_required(VERSION 3.18)
project(rus_sim_bringup)

find_package(ament_cmake REQUIRED)

# ── 安装 launch 文件 ──
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
