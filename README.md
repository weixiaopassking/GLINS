# AlkaidQuadrotor

**项目说明** 写着玩的demo，从零搭建并手写实现一个智能无人机(决策、规划、感知、通讯、GUI), 主要不断整合并竞赛、实习、课程时所学到的工程技巧及算法 ,侧重于工程架构、算法实现、性能效率 

**更新说明** 稳定版义tag形式发布，测试节点以branch形式发布，如有问题可提交issue或者发邮箱niu_wengang@163.com       



## 1.算法流程

<div align=center><img src="./file/pic/framework_software.drawio.svg" style="zoom:100%;" ></div>

### 1.1 定位

LIO版<sup>[1]</sup><sup>[2]</sup>

### 1.2 规划
hybrid+minimum snap <sup>[3]</sup>


### 1.3 感知


### 1.4决策
决策树

## 2.硬件配置

|      器件       | 数量 | 价格 |
| :------: | :--: | :----: |
|    QV250机架及配件    |  1   | 100 |
| EMAX Bullet 30A |  4   | 510 |
|     R12DSM 接收机     |  1   | 90 |
|  livox mid360 及配件  |  1   | 5000 |
| realsense d435i | 1 | 2700 |
| 乐迪 AT9S Pro | 1 | 600 |
| Holybro Pixhawk 6CMini飞控 | 1 | 1200 |
| BB响 | 1 | 10 |
| 格氏锂电池 4s 2300mah | 1 | 140 |
| IMAX B6AC 80W平衡充 | 1 | 120 |
| DCDC降压 | 2 | 30 |
| TMOTOR V2306-2400KV | 5 | 350 |
| 工具及耗材 | — | 400 |
| 合计 |  | 11250 |

<div align=center><img src="./file/pic/framework_hardware.drawio.svg" style="zoom:100%;" ></divdiv>

## 3.环境配置

cmake/thirdparty_libs.cmake

|  开源库  |  作用  |    版本    |安装方式|版本查看|
| :----: | :----: | :----: | :----: | ------ |
| eigen | 矩阵运算 | 3.3.4 |```sudo apt-get install libeigen3-dev```|```pkg-config --modversion eigen3```|
| pcl | 3D点云处理 |                | ```sudo apt-get install ros-melodic-pcl-ros``` |  |
| opencv |     2D图像处理      |3.2.0| ```sudo apt-get install libopencv-dev``` | ```pkg-config opencv --modversion``` |
|  gtest   |      单元测试       |                |    ```sudo apt-get install libgtest-dev```     |                                      |
| yaml-cpp |      yaml读写       |                |    `sudo apt-get install libyaml-cpp-dev`    |                                      |
| geographiclib | gnss处理 |                | source code |                                      |
|  sophus  |   矩阵运算(流形)    || source code | |
| tbb | Intel的并行加速框架 |oneTBB-2019_U8| source code | |




## 4.运行
### 4.1 数据集

+ NCLT

+ Urban Nav
+ 自采数据(no gt)

### 4.2 启动




## 5.更新日志

|日期| Branch | 更新 |说明 | commit_id | video |
| :----: | :----:| :----: | :----: | ------ | ------ |
| 2022//11/20 | V0.1 | gnss imu lidar松耦合建图与定位简易demo |仅数据集验证,侧重框架构建|6885639|[v0.1 demo1](https://www.bilibili.com/video/BV1mt4y1K7Nt/?spm_id_from=333.999.0.0&vd_source=b86740d9f2b244ac781ad5f60dd8e818)     [v0.1 demo2](https://www.bilibili.com/video/BV1Ce4y1s75g/?spm_id_from=333.788&vd_source=b86740d9f2b244ac781ad5f60dd8e818)|
| 2023/08/15 | V0.2 | 重构框架, | 新框架实际设备手持 |                                    |                                    |
|  | V0.3 |      ||||









## 5.标准与规范

+ **C++标准** c++ 17   
+ **代码格式风格** visual studio风格  
 + **命名规范**  
    类 MyClass  
    函数 MyFunction      
    类内变量 _my_function_ptr _my_function_vec    
    普通变量 my_function_ptr   my_function_vec    
 +  **commit规范**<sup>[4]</sup>

```Bash
<type>[<scope>]:<subject>
```

--type

feat：新功能  
fix：已修复  
to：修复中  
docs：文档  
style：格式  
refactor：重构  
perf：优化  
config: 修改配置文件  
test：增加测试  
chore：构建过程或辅助工具的变动  
revert：回滚到上一个版本  
merge：代码合并  
sync：同步主线或分支的Bu  

例如:
```
fix[Planning]:修复机身卡膨胀层静止的bug  
docs[README]:增加参考文章链接  
feat[Localization]:增加kd tree方法  
```

## 6.参考

[1]  [MARS:ROG-Map](https://github.com/hku-mars/ROG-Map)  
[2]  [gaoxiang:slam_in_autonomous_driving](https://github.com/gaoxiang12/slam_in_autonomous_driving)  
[3]  [ HUKST Aerial Robotics Group:Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)  

[4]  [阿里开发者:如何规范你的Git commit？](https://zhuanlan.zhihu.com/p/182553920)  

## 7.许可证
遵循 GPLv3.
