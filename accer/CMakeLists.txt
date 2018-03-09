project(accer)
cmake_minimum_required(VERSION 2.8)
# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

add_executable( accer main.cpp )
# 与Ceres和OpenCV链接
target_link_libraries( accer ${CERES_LIBRARIES} )



