# AlkaidQuadrotor

**项目说明** 写着玩的demo，从零搭建并手写实现一个智能无人机(决策、规划、感知、通讯、GUI), 主要不断整合并竞赛、实习、课程时所学到的工程技巧及算法 ,侧重于工程架构、算法实现、性能效率 

**更新说明** 稳定版义tag形式发布，测试节点以branch形式发布，如有问题可提交issue或者发邮箱niu_wengang@163.com       




## 1.算法流程

### 1.1 定位

LIO版

### 1.2 规划
hybrid+minimum snap


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



## 3.环境配置
|  开源库  |  作用  |    版本    |安装方式|
| :----: | :----: | :----: | :----: |
| yaml-cpp | 读yaml文件参数 |0.6.0|  |
| glog | 日志 ||  |
| tbb | cpu并行计算 || |
| pcl | 点云处理 || |
| gflags | 读命令行参数 || |
|  |  || |



## 4.更新日志

|日期| Branch | 内容 |说明 | commit_id | video |
| :----: | :----:| :----: | :----: | ------ | ------ |
| 2022//11/20 | V0.1 | gnss imu lidar松耦合建图与定位 |简单demo,仅数据集验证,侧重框架构建|6885639|[v0.1 demo1](https://www.bilibili.com/video/BV1mt4y1K7Nt/?spm_id_from=333.999.0.0&vd_source=b86740d9f2b244ac781ad5f60dd8e818)     [v0.1 demo2](https://www.bilibili.com/video/BV1Ce4y1s75g/?spm_id_from=333.788&vd_source=b86740d9f2b244ac781ad5f60dd8e818)|
| 2023/07/30 | V0.2 |  |                                    |                                    |                                    |
|  | V0.3 |      ||||









## 5.标准与规范

**C++标准** c++ 17   
**代码格式风格** visual studio风格
 **命名规范**
类 MyClass   如何规范你的Git commit？
函数 MyFunction    
类内变量 _my_function_ptr _my_function_vec  
普通变量 my_function_ptr   my_function_vec  
**commit规范**[1]

```Bash
<type>[<scope>]:<subject>
```

--type

feat：新功能（feature）
fix/to：修复bug，可以是QA发现的BUG，也可以是研发自己发现的BUG
fix：产生diff并自动修复此问题。适合于一次提交直接修复问题
to：只产生diff不自动修复此问题。适合于多次提交。最终修复问题提交时使用fix
docs：文档（documentation）
style：格式（不影响代码运行的变动）
refactor：重构（即不是新增功能，也不是修改bug的代码变动）
perf：优化相关，比如提升性能、体验
test：增加测试。
chore：构建过程或辅助工具的变动
revert：回滚到上一个版本
merge：代码合并。
sync：同步主线或分支的Bu

例如:
```
fix(Planning):修复机身卡膨胀层静止的bug  
docs(README):增加参考文章链接  
feat(Localization):增加kd tree方法  
```

## 6.参考

[1]  [如何规范你的Git commit？](https://zhuanlan.zhihu.com/p/182553920)



## 7.许可证
遵循 GPLv3.
