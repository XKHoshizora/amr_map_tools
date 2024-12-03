# AMR Map Tools API 中文文档

## 简介

AMR Map Tools 是一个用于自主移动机器人的 ROS 功能包，提供了航点管理和导航功能。该包包含了管理航点和充电桩、执行导航任务等核心功能。

## 服务定义

### GetNumOfWaypoints.srv

```
---
int32 num
```

返回系统中航点或充电桩的总数。

### GetWaypointByIndex.srv

```
int32 index
---
string name
geometry_msgs/Pose pose
```

通过索引获取航点或充电桩信息。

### GetWaypointByName.srv

```
string name
---
string name
geometry_msgs/Pose pose
```

通过名称获取航点或充电桩信息。

## 消息定义

### Waypoint.msg

```
string frame_id      # 航点的参考坐标系
string name         # 航点的名称标识符
geometry_msgs/Pose pose  # 航点的位置和方向
```

## 核心服务

### 航点管理服务

#### 获取航点数量

- **服务名称**: `waypoint/get_num_waypoint`
- **服务类型**: `GetNumOfWaypoints`
- **功能描述**: 返回已存储的航点总数
- **响应**:
  - `num` (int32): 航点数量

#### 通过索引获取航点

- **服务名称**: `waypoint/get_waypoint_index`
- **服务类型**: `GetWaypointByIndex`
- **请求**:
  - `index` (int32): 航点索引
- **响应**:
  - `name` (string): 航点名称
  - `pose` (geometry_msgs/Pose): 航点在地图坐标系中的位姿

#### 通过名称获取航点

- **服务名称**: `waypoint/get_waypoint_name`
- **服务类型**: `GetWaypointByName`
- **请求**:
  - `name` (string): 航点名称
- **响应**:
  - `name` (string): 航点名称（确认）
  - `pose` (geometry_msgs/Pose): 航点在地图坐标系中的位姿

### 充电桩管理服务

#### 获取充电桩数量

- **服务名称**: `waypoint/get_num_charger`
- **服务类型**: `GetNumOfWaypoints`
- **功能描述**: 返回已存储的充电桩总数
- **响应**:
  - `num` (int32): 充电桩数量

#### 通过索引获取充电桩

- **服务名称**: `waypoint/get_charger_index`
- **服务类型**: `GetWaypointByIndex`
- **请求**:
  - `index` (int32): 充电桩索引
- **响应**:
  - `name` (string): 充电桩名称
  - `pose` (geometry_msgs/Pose): 充电桩在地图坐标系中的位姿

#### 通过名称获取充电桩

- **服务名称**: `waypoint/get_charger_name`
- **服务类型**: `GetWaypointByName`
- **请求**:
  - `name` (string): 充电桩名称
- **响应**:
  - `name` (string): 充电桩名称
  - `pose` (geometry_msgs/Pose): 充电桩在地图坐标系中的位姿

## 话题

### 发布者

#### 航点可视化

- **话题**: `waypoints_marker`
- **类型**: `visualization_msgs/Marker`
- **描述**: 在 RViz 中发布航点的可视化标记

#### 充电桩可视化

- **话题**: `chargers_marker`
- **类型**: `visualization_msgs/Marker`
- **描述**: 在 RViz 中发布充电桩的可视化标记

#### 导航结果

- **话题**: `waypoint/navi_result`
- **类型**: `std_msgs/String`
- **描述**: 发布导航结果（"done"或"failure"）

### 订阅者

#### 添加航点

- **话题**: `waypoint/add_waypoint`
- **类型**: `amr_waypoint_tools/Waypoint`
- **描述**: 向系统添加新的航点

#### 添加充电桩

- **话题**: `waypoint/add_charger`
- **类型**: `amr_waypoint_tools/Waypoint`
- **描述**: 向系统添加新的充电桩

#### 导航到航点

- **话题**: `waypoint/navi_waypoint`
- **类型**: `std_msgs/String`
- **描述**: 命令机器人导航到指定名称的航点

#### 导航到指定位姿

- **话题**: `waypoint/navi_pose`
- **类型**: `geometry_msgs/Pose`
- **描述**: 命令机器人导航到指定的位置和姿态

## 节点

### wp_manager

航点和充电桩管理器，提供访问和保存航点的服务。

### wp_navi_server

处理到指定航点名称的导航请求。

### pose_navi_server

处理到指定位姿的导航请求。

### wp_nav_remote

提供远程控制航点导航的功能。

### wp_nav_odom_report

通过 UDP 向远程服务器报告机器人位置。

- **参数**:

  - Robot_ID (int): 默认值 = 1
  - 服务器 IP: 默认值 = "192.168.1.110"
  - 服务器端口: 默认值 = 20180

- **订阅的坐标转换**:

  - `map` → `base_footprint`

- **功能**:
  - 以 10Hz 的频率报告机器人位置(x, y)和方向(yaw)
  - 通过 UDP 向远程服务器发送位置更新
  - 使用 TF 获取机器人在地图坐标系中的位姿

## 文件格式

航点和充电桩信息以 XML 格式存储，结构如下：

```xml
<Waterplus>
  <Waypoint>
    <Name>航点名称</Name>
    <Pos_x>x坐标</Pos_x>
    <Pos_y>y坐标</Pos_y>
    <Pos_z>z坐标</Pos_z>
    <Ori_x>方向四元数x</Ori_x>
    <Ori_y>方向四元数y</Ori_y>
    <Ori_z>方向四元数z</Ori_z>
    <Ori_w>方向四元数w</Ori_w>
  </Waypoint>
  <Charger>
    <!-- 与Waypoint结构相同 -->
  </Charger>
</Waterplus>
```

## 使用示例

### 读取航点信息

```cpp
// 获取航点数量
ros::ServiceClient client = nh.serviceClient<amr_waypoint_tools::GetNumOfWaypoints>("waypoint/get_num_waypoint");
amr_waypoint_tools::GetNumOfWaypoints srv;
if(client.call(srv)) {
    int num_waypoints = srv.response.num;
}

// 通过名称获取航点
ros::ServiceClient client = nh.serviceClient<amr_waypoint_tools::GetWaypointByName>("waypoint/get_waypoint_name");
amr_waypoint_tools::GetWaypointByName srv;
srv.request.name = "waypoint1";
if(client.call(srv)) {
    geometry_msgs::Pose pose = srv.response.pose;
    // 使用航点位姿
}

// 通过索引获取航点
ros::ServiceClient client = nh.serviceClient<amr_waypoint_tools::GetWaypointByIndex>("waypoint/get_waypoint_index");
amr_waypoint_tools::GetWaypointByIndex srv;
srv.request.index = 0;
if(client.call(srv)) {
    std::string name = srv.response.name;
    geometry_msgs::Pose pose = srv.response.pose;
    // 使用航点信息
}
```

### 发布新的航点

```cpp
ros::Publisher wp_pub = nh.advertise<amr_waypoint_tools::Waypoint>("waypoint/add_waypoint", 1);
amr_waypoint_tools::Waypoint wp;
wp.frame_id = "map";
wp.name = "new_waypoint";
wp.pose.position.x = 1.0;
wp.pose.position.y = 2.0;
wp.pose.orientation.w = 1.0;
wp_pub.publish(wp);
```

### 导航控制

```cpp
// 导航到指定名称的航点
ros::Publisher pub = nh.advertise<std_msgs::String>("waypoint/navi_waypoint", 1);
std_msgs::String msg;
msg.data = "waypoint1";
pub.publish(msg);

// 导航到指定位姿
ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("waypoint/navi_pose", 1);
geometry_msgs::Pose pose;
pose.position.x = 1.0;
pose.position.y = 2.0;
pose.orientation.w = 1.0;
pub.publish(pose);
```


以最新的这几个代码文件为基础，检查整体逻辑是否已经涵盖了所有导航模式的逻辑？

假设：
waypoint_names: ['1', '2', '3', '4']（航点名称列表）
start_index: 1（可以是航点列表中的任意位置，如1, 0, 2, ...）

1. 单点导航（SINGLE_POSE/SINGLE_WAYPOINT）：
到达目标点后完成导航

2. 单程导航（SEQUENCE_ONCE）：
- 正向：从start_index到序列末尾，"2" → "3"  → "4"  → 停止
- 反向：从start_index到序列开始，"2" → "1" → 停止

3. 循环导航（SEQUENCE_LOOP）：
- 正向：从start_index到达末尾后回到开始继续，"2" → "3"  → "4"  → "2"  → "3"  → "4" ...
- 反向：从start_index到达开始后回到末尾继续，"2" → "1"  → "2"  → "1"  → "2" ...

4. 往返导航（SEQUENCE_BACK_FORTH）：
- 去程阶段：
  - 正向：从start_index到末尾，"2" → "3"  → "4"
  - 反向：从start_index到开始，"2" → "1"
- 返程阶段：
  - 正向：从末尾返回到开始，"4"  → "3"  → "2"
  - 反向：从开始返回到末尾，"1"  → "2"
如此往复


检查是否可以完成以下导航控制功能，且控制逻辑是否完整？

1. 单点导航：导航到指定航点或指定坐标。
- 指定航点名称：指定非空航点列表中航点的名称，使对应的航点作为导航目标点。
- 指定航点索引：指定非空航点列表中航点的索引，使对应的航点作为导航目标点。
- 指定目标点坐标：与是否存在航点列表无关，均可通过输入 `(x, y)` 坐标和自定义的 `yaw` 角，使对应的坐标点和朝向作为导航目标点。

2. 单程导航：指定起始航点，然后根据指定的单程模式（正向、反向）导航。

3. 循环导航：指定起始航点，然后根据指定的循环模式（正向、反向）导航。

4. 往返导航：指定起始航点，然后根据指定的往返模式（正向、反向）导航。

5. 暂停：保存当前导航任务（导航模式，起始航点，当前航点，导航序列，导航索引等等必要的数据，保证恢复导航时可以完全恢复到暂停之前的导航状态），并暂停导航，直到恢复导航。

6. 恢复：通过暂停时保存的导航任务信息，完全恢复到暂停之前的导航状态，继续执行导航任务。

7. 停止：立即停止导航，并取消所有导航任务，将状态和序列等等导航信息初始化，以便再次开始导航任务。

8. 切换导航模式：在任何导航模式下，都可以直接取消当前导航任务，然后立刻无缝切换为不同的导航模式（单点导航、单程导航、循环导航、往返导航等）。

9. 退出：退出导航控制节点。



rostopic pub /wp_nav_controller/waypoint_sequence wp_nav_controller/WaypointSequence "waypoint_names: ['1', '2', '3', '4']
mode: 2
direction: 1
start_index: 3" -1