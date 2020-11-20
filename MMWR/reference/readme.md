## USBCAN-II资料下载
https://www.zlg.cn/can/down/down/id/35.html

## USBCAN-II新版驱动安装
USBCAN-II新版驱动基于libusb实现，请确保运行环境中有libusb-1.0的库，可连网在线安装，命令如下：
apt-get install libusb-1.0-0

将libusbcan.so拷到/lib目录

## 在ROS中调用外部链接库文件(.so)配置方法
１、建立文件树如下：
myproject:
-include

--Interface.h
-lib

--libInterface.so
-src

--control.cpp
-CMakeLists.txt
-package.xml
在对应的源文件包下建立两个文件夹include(放.so对应的头文件)，lib(放.so文件)

2、配置CMakeLists.txt
include_directories(
  ${catkin_INCLUDE_DIRS} include
)

link_directories(
  ${catkin_LIB_DIRS} lib
)

add_executable(base_control src/control.cpp)
target_link_libraries(base_control
  ${catkin_LIBRARIES} Interface
)
注意最后倒数第二行Interface这里，该.so文件名为libInterface.so

