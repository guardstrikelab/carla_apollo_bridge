# Deployment Introduction

To make the Carla Apollo Bridge work properly, you need to start the following three containers:

**Apollo container**

```bash
$ bash docker/scripts/dev_start.sh
$ bash docker/scripts/dev_into.sh
//in container
./apollo.sh build_gpu
./scripts/bootstrap.sh
```

The above commands start Apollo/CyberRT environment and Dreamview service, where CyberRT HOST(CYBER\_IP) is the external address of the container. Since the container is started with HOST network, in Ubuntu, the CYBER\_IP is 172.17.0.1(docker0)

**Carla container**

```bash
cd carla_apollo_bridge/scripts
./docker_run_carla.sh
```

The preceding command starts a Carla container with HOST network. You can access the Carla service through docker0(172.17.0.1).

**Carla\_Apollo\_Bridge Container**
```bash
cd carla_apollo_bridge/docker
./build_docker.sh
./run_docker.sh
docker exec -ti carla_cyber_0.9.14 bash
```
The preceding command starts Carla Apollo Bridge container. By default, the container uses the Docker Bridge Network and its external address is 172.17.0.X. The container uses the CyberRT environment, which is in the same Cyber Network as the Apollo container. They can discover and communicate with each other.

To sum up, its network architecture is as follows:

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/XJ9LnWvpz0eZlvDe/img/dbc28719-8e19-4311-a9f4-01ceb490ba4a.png)

**Other**

Execution in Apollo container <code>./scripts/bootstrap.sh</code> will start the Dreamview service, Dreamview service is provided by CyberRT Node, this can be check with <code>cyber\_node list</code> in CyberRT environment (Apollo container or carla apollo bridge container)

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/XJ9LnWvpz0eZlvDe/img/f172e57e-b22a-4a8c-b1f3-39ff0bf12a00.png)

Dreamview UI and Dreamview services communicate through Websocket.

### Reference:

##### [**Cyber RT Topology Discovery**](https://developer.baidu.com/article/detail.html?id=290450)

##### [**CyberRT basic framework**](https://zhuanlan.zhihu.com/p/479518561)

#### [**Baidu self-driving simulation**](https://zhuanlan.zhihu.com/p/398401943)