该文件夹为AMBER B1的UDP通信协议开发文档里，示例代码名称中的数字即为各命令编号。

UDP通信协议文档参见：https://github.com/MrAsana/AMBER_B1_ROS2/wiki/SDK-&-API---UDP-Ethernet-Protocol--for-controlling-&-programing
其中gui_5_node的代码中包含了机械臂动态避障的部分(rrt算法进行末端避碰+障碍检测+执行)

其中，rrt算法寻路部分为RRT::planning

障碍检测   七轴机械臂关节RRT::collisionCheck2    机械臂末端障碍检测RRT::collisionCheck

均可根据自己的需要修改

使用方法（以amber_gui_5_node为例子）：

1、进入路径/amber_gui_5_node/cmake-build-debug/

2、打开终端

3、终端中输入"【】"中的内容：【./amber_gui_5_node】，如果出现问题，请按以下顺序执行：

  3.1删除文件夹内/amber_gui_5_node/cmake-build-debug/的所有文件
  
  3.2终端中输入"【】"中的内容：【cmake ..】
  
  3.3终端中输入"【】"中的内容：【make】
  
  3.4终端中输入"【】"中的内容：【./amber_gui_5_node】

testtesttest
