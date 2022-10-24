# 从0到1自动驾驶多传感器融合定位
## 1.项目介绍
自动驾驶多传感器融合定位的个人学习记录，将会从0到1逐步实现一个完整且实用的建图、定位框架并将根据实际需求不断魔改  
阶段性代码将以Tag形式记录,10days左右上传新tag,如遇问题可提交issue :relaxed:  
感谢任乾、李想等大佬的开源贡献,受益良多  

## 2.主要内容

+ [x] Tag v1.0 惯导数据可视化     
+ [x] Tag v2.0 ndt里程计    
+ [x] Tag v3.0 ndt里程计松耦合gnss  
+ [ ] Tag v4.0 带闭环的ndt里程计松耦合gnss  
+ [ ] Tag v5.0 重定位功能  
+ [ ] Tag v6.0 带闭环的fastlio2里程计松耦合gnss  
 


流程及代码框架可见[博客:多传感器融合定位学习系列](https://blog.csdn.net/weixin_37684239/article/details/126571774?spm=1001.2014.3001.5502) (PS: 咕咕咕 尽量及时更新不鸽 :laughing:) 

## 3.环境依赖
+ ubuntu18.04 
+ ROS melodic  
+ yaml-cpp >=6.0  
+ google glog  
+ geographicLib
+ g2o  

安装方法可参考[博客:机器人开发常见第三方库、软件安装和使用](https://blog.csdn.net/weixin_37684239/article/details/126568335?spm=1001.2014.3001.5501)

## 4.如何运行
见multisensor_localization功能包下readme文件夹

## 5 工程适配
 **前端里程计**
+ [x] NDT    
+ [ ] ICP
+ [ ] Fast lio2  
+ [ ] Aloam  

**后端优化器**
+ [x] g2o
+ [ ] ceres
+ [ ] gtsam

**回环检测**
+ [ ] ndt    
+ [ ] Scan Context    

**重定位**
+ [ ] DNT  

**数据集**
+ [x] kitti      
+ [ ] kaist    
 
## 6.参考
[任乾 知乎专栏从零开始做自动驾驶](https://zhuanlan.zhihu.com/p/83775731)  
[李太白lx 从零开始学定位 ](https://blog.csdn.net/tiancailx/article/details/125785641?spm=1001.2014.3001.5501)  
深蓝学院 多传感器融合定位      
github.com/XiaotaoGuo/modular_mapping_and_localization_framework
