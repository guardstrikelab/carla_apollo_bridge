
# Apollo 7.0 和  Carla 0.9.13 联合仿真





# 1. 编译运行 Apollo
本机配置要求参考：[https://github.com/ApolloAuto/apollo](https://github.com/ApolloAuto/apollo)

请下载 apollo 官方代码，本 repo 在 Apollo  master 分支做了以下修改：

## 1.1 perception模块启动问题

1. 将 apollo/modules/perception/production/conf/perception/perception_common.flag 中的
```shell
--obs_tf2_buff_size=0.01
```
修改为
```shell
--obs_tf2_buff_size=0.1
```
参考：[https://github.com/ApolloAuto/apollo/issues/12088#issuecomment-673750020](https://github.com/ApolloAuto/apollo/issues/12088#issuecomment-673750020)

2. 将 apollo/modules/drivers/lidar/velodyne/params/velodyne128_novatel_extrinsics.yaml里面的transform改为：
```shell
translation:
    x: 0
    y: 0
    z: 2.5
  rotation:
    x: 0
    y: 0
    z: 0
    w: 1
```
参考：[add extrinsics file for lidar so perception worksfc622d1](https://github.com/ApolloAuto/apollo/commit/fc622d1e514b721b2030cb7df575aa313100f6db)

## 1.2 交通灯识别问题
### 1.2.1 延迟调整
apollo的modules/perception/production/conf/perception/camera/trafficlights_perception_component.config中：
```shell
tf2_timeout_second : 0.5
```

### 1.2.2 图片格式大小调整
从Carla传来的图片格式和大小不对。在apollo modules/perception/onboard/component/trafficlights_perception_component.h第158行规定了图片大小为1920*1080.

在carla-apollo里面的carla_spawn_objects/config/objects.json的相机配置部分，修改图像大小
```shell
"image_size_x": 1920,
"image_size_y": 1080,
```

在carla-apollo里面的carla_cyber_bridge/camera.py，把140左右的self.camera_image_writer.write(cam_img)注释。相当于不给 /apollo/sensor/camera/front_12mm/image、/apollo/sensor/camera/front_6mm/image发数据。

启动bridge等之后，在apollo容器中运行
```shell
cyber_launch start modules/drivers/tools/image_decompress/launch/image_decompress.launch

```
这个会读取/apollo/sensor/camera/front_12mm/image/compressed的数据，把图片处理为**rgb8**的格式，发送给/apollo/sensor/camera/front_12mm/image。6mm同理。
此时查看monitor的/apollo/sensor/camera/front_12mm/image，可以发现encoding为rgb8，data在不断变化。使用cyber_visualizer，点击Add Image，ChannelName选择monitor的/apollo/sensor/camera/front_12mm/image，即可预览颜色正常的图像

### 1.2.3 图片坐标映射问题
注：这不一定是最终的解决方案，后续还需要根据坐标转换原理考察这个改动是否合理
在modules/perception/camera/lib/traffic_light/preprocessor/multi_camera_projection.cc L178加入
```cpp
int tmp = min_y;
min_y = -1*max_y;
max_y = -1*tmp;
```
![image.png](https://cdn.nlark.com/yuque/0/2022/png/703243/1666332370045-067489a3-7551-475b-aa26-c10859debd41.png#clientId=u5dc14aca-4765-4&crop=0&crop=0&crop=1&crop=1&from=paste&height=400&id=u063d7446&margin=%5Bobject%20Object%5D&name=image.png&originHeight=400&originWidth=810&originalType=binary&ratio=1&rotation=0&showTitle=false&size=71396&status=done&style=none&taskId=u0767946b-45d6-45ff-a716-2a27557f7e7&title=&width=810)



依次执行以下命令：
```shell
cd apollo_carla/apollo
echo "export APOLLO_ROOT_DIR=$(pwd)" >> ~/.bashrc  && source ~/.bashrc
bash docker/scripts/dev_start.sh
```
执行成功后会有下面的输出：
```shell
[ OK ] Congratulations! You have successfully finished setting up Apollo Dev Environment.
[ OK ] To login into the newly created apollo_dev_lei container, please run the following command:
[ OK ]   bash docker/scripts/dev_into.sh
[ OK ] Enjoy!
```
执行命令，进入容器
```shell
bash docker/scripts/dev_into.sh
```
编译 GPU 版本
```shell
./apollo.sh build_gpu
```
编译成功后有如下打印：
```shell
==============================================
[ OK ] Done building apollo. Enjoy!
==============================================

```
在容器中执行
```shell
./scripts/bootstrap.sh
```
然后在浏览器中打开  localhost:8888

# 2. 启动 Carla
```shell
cd apollo_carla/carla
xhost +
./docker_run_carla.sh
```

# 3. 运行 carla_apollo_bridge
## 3.1 启动
```shell
cd apollo_carla/carla_apollo_bridge/docker

./build_docker.sh

./run_docker.sh
```
## 3.2 配置
可以在容器外面通过下面命令来查询 `carla_cyber_0.9.13`容器 `IP` 地址
```shell
docker inspect carla_cyber_0.9.13 | grep IPAddress
```
然后进入容器
```shell
docker exec -ti carla_cyber_0.9.13 bash
vi /apollo/cyber/setup.bash
```
修改 `/apollo/cyber/setup.bash`文件中的 `CYBER_IP` 为本机 `carla_cyber_0.9.13` 容器 `IP` 地址，即上面查询出来的地址。
```shell
export CYBER_IP=172.17.0.2
```
在容器中更新环境变量
```shell
source ~/.bashrc
```
## 3.3 编译
在 `carla_cyber_0.9.13` 容器中执行
```shell
rm -rf /root/.cache/*
./apollo.sh build_cyber opt
```
编译成功打印
```shell
[INFO] Skipping revision recording
============================
[ OK ] Build passed!
[INFO] Took 61 seconds

```
## 3.4 运行
在  `carla_cyber_0.9.13` 容器中运行以下命令。
新开一个终端窗口，执行
```shell
cd /apollo/cyber/carla_bridge
python carla_cyber_bridge/bridge.py
```
另外新开一个终端窗口，执行
```shell
cd /apollo/cyber/carla_bridge
python carla_spawn_objects/carla_spawn_objects.py
```

# 鸣谢
备注：该联合仿真是在以下基础上改动的，在此表示感谢。
[https://github.com/casper-auto/carla-apollo](https://github.com/casper-auto/carla-apollo)
[https://github.com/AuroAi/carla_apollo_bridge](https://github.com/AuroAi/carla_apollo_bridge)
[https://github.com/ApolloAuto/apollo](https://github.com/ApolloAuto/apollo)





