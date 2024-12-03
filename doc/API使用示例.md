# AMR Map Tools API 使用示例文档

## 1. 获取航点数量服务示例

### C++ 示例

```cpp
#include <ros/ros.h>
#include <amr_waypoint_tools/GetNumOfWaypoints.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_counter_node");
    ros::NodeHandle nh;

    // 创建服务客户端
    ros::ServiceClient client = nh.serviceClient<amr_waypoint_tools::GetNumOfWaypoints>("waypoint/get_num_waypoint");

    // 创建服务请求
    amr_waypoint_tools::GetNumOfWaypoints srv;

    // 调用服务
    if (client.call(srv)) {
        ROS_INFO("航点总数: %d", srv.response.num);
    } else {
        ROS_ERROR("调用服务失败");
    }
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python
import rospy
from amr_waypoint_tools.srv import GetNumOfWaypoints

def get_waypoint_count():
    rospy.init_node('waypoint_counter_node', anonymous=True)

    # 等待服务可用
    rospy.wait_for_service('waypoint/get_num_waypoint')
    try:
        # 创建服务代理
        get_num = rospy.ServiceProxy('waypoint/get_num_waypoint', GetNumOfWaypoints)

        # 调用服务
        response = get_num()
        rospy.loginfo("航点总数: %d", response.num)
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

if __name__ == "__main__":
    get_waypoint_count()
```

## 2. 通过索引获取航点服务示例

### C++ 示例

```cpp
#include <ros/ros.h>
#include <amr_waypoint_tools/GetWaypointByIndex.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_getter_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<amr_waypoint_tools::GetWaypointByIndex>("waypoint/get_waypoint_index");

    amr_waypoint_tools::GetWaypointByIndex srv;
    srv.request.index = 0;  // 获取第一个航点

    if (client.call(srv)) {
        ROS_INFO("航点名称: %s", srv.response.name.c_str());
        ROS_INFO("位置: (%.2f, %.2f, %.2f)",
            srv.response.pose.position.x,
            srv.response.pose.position.y,
            srv.response.pose.position.z);
    } else {
        ROS_ERROR("调用服务失败");
    }
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python
import rospy
from amr_waypoint_tools.srv import GetWaypointByIndex

def get_waypoint_by_index(index):
    rospy.init_node('waypoint_getter_node', anonymous=True)

    rospy.wait_for_service('waypoint/get_waypoint_index')
    try:
        get_wp = rospy.ServiceProxy('waypoint/get_waypoint_index', GetWaypointByIndex)
        response = get_wp(index)
        rospy.loginfo("航点名称: %s", response.name)
        rospy.loginfo("位置: (%.2f, %.2f, %.2f)",
                     response.pose.position.x,
                     response.pose.position.y,
                     response.pose.position.z)
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

if __name__ == "__main__":
    get_waypoint_by_index(0)  # 获取第一个航点
```

## 3. 通过名称获取航点服务示例

### C++ 示例

```cpp
#include <ros/ros.h>
#include <amr_waypoint_tools/GetWaypointByName.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_name_getter_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<amr_waypoint_tools::GetWaypointByName>("waypoint/get_waypoint_name");

    amr_waypoint_tools::GetWaypointByName srv;
    srv.request.name = "kitchen";  // 获取名为"kitchen"的航点

    if (client.call(srv)) {
        ROS_INFO("找到航点: %s", srv.response.name.c_str());
        ROS_INFO("位置: (%.2f, %.2f, %.2f)",
            srv.response.pose.position.x,
            srv.response.pose.position.y,
            srv.response.pose.position.z);
    } else {
        ROS_ERROR("调用服务失败");
    }
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python
import rospy
from amr_waypoint_tools.srv import GetWaypointByName

def get_waypoint_by_name(name):
    rospy.init_node('waypoint_name_getter_node', anonymous=True)

    rospy.wait_for_service('waypoint/get_waypoint_name')
    try:
        get_wp = rospy.ServiceProxy('waypoint/get_waypoint_name', GetWaypointByName)
        response = get_wp(name)
        rospy.loginfo("找到航点: %s", response.name)
        rospy.loginfo("位置: (%.2f, %.2f, %.2f)",
                     response.pose.position.x,
                     response.pose.position.y,
                     response.pose.position.z)
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

if __name__ == "__main__":
    get_waypoint_by_name("kitchen")  # 获取名为"kitchen"的航点
```

## 4. 添加新航点示例

### C++ 示例

```cpp
#include <ros/ros.h>
#include <amr_waypoint_tools/Waypoint.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_adder_node");
    ros::NodeHandle nh;

    ros::Publisher wp_pub = nh.advertise<amr_waypoint_tools::Waypoint>("waypoint/add_waypoint", 1);

    amr_waypoint_tools::Waypoint wp;
    wp.frame_id = "map";
    wp.name = "new_kitchen";
    wp.pose.position.x = 2.0;
    wp.pose.position.y = 3.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.w = 1.0;

    ros::Rate rate(1);
    while(ros::ok()) {
        wp_pub.publish(wp);
        ROS_INFO("发布新航点");
        rate.sleep();
    }
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python
import rospy
from amr_waypoint_tools.msg import Waypoint
from geometry_msgs.msg import Pose

def add_new_waypoint():
    rospy.init_node('waypoint_adder_node', anonymous=True)
    wp_pub = rospy.Publisher('waypoint/add_waypoint', Waypoint, queue_size=1)

    wp = Waypoint()
    wp.frame_id = "map"
    wp.name = "new_kitchen"
    wp.pose = Pose()
    wp.pose.position.x = 2.0
    wp.pose.position.y = 3.0
    wp.pose.position.z = 0.0
    wp.pose.orientation.w = 1.0

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        wp_pub.publish(wp)
        rospy.loginfo("发布新航点")
        rate.sleep()

if __name__ == "__main__":
    try:
        add_new_waypoint()
    except rospy.ROSInterruptException:
        pass
```

## 5. 导航控制示例

### C++ 示例

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

class NavigationController {
public:
    NavigationController() {
        // 导航到命名航点的发布者
        wp_nav_pub = nh.advertise<std_msgs::String>("waypoint/navi_waypoint", 1);
        // 导航到指定位姿的发布者
        pose_nav_pub = nh.advertise<geometry_msgs::Pose>("waypoint/navi_pose", 1);
    }

    // 导航到指定名称的航点
    void navigateToWaypoint(const std::string& wp_name) {
        std_msgs::String msg;
        msg.data = wp_name;
        wp_nav_pub.publish(msg);
        ROS_INFO("导航到航点: %s", wp_name.c_str());
    }

    // 导航到指定位姿
    void navigateToPose(float x, float y, float theta) {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.orientation.w = cos(theta/2);
        pose.orientation.z = sin(theta/2);
        pose_nav_pub.publish(pose);
        ROS_INFO("导航到位置: (%.2f, %.2f), 朝向: %.2f", x, y, theta);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher wp_nav_pub;
    ros::Publisher pose_nav_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_controller_node");

    NavigationController controller;

    // 导航到名为"kitchen"的航点
    controller.navigateToWaypoint("kitchen");

    ros::Duration(5.0).sleep();  // 等待5秒

    // 导航到指定位置
    controller.navigateToPose(2.0, 3.0, 1.57);  // x=2.0, y=3.0, theta=90度

    ros::spin();
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class NavigationController:
    def __init__(self):
        # 导航到命名航点的发布者
        self.wp_nav_pub = rospy.Publisher('waypoint/navi_waypoint',
                                        String, queue_size=1)
        # 导航到指定位姿的发布者
        self.pose_nav_pub = rospy.Publisher('waypoint/navi_pose',
                                          Pose, queue_size=1)

    def navigate_to_waypoint(self, wp_name):
        msg = String()
        msg.data = wp_name
        self.wp_nav_pub.publish(msg)
        rospy.loginfo("导航到航点: %s", wp_name)

    def navigate_to_pose(self, x, y, theta):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.w = math.cos(theta/2)
        pose.orientation.z = math.sin(theta/2)
        self.pose_nav_pub.publish(pose)
        rospy.loginfo("导航到位置: (%.2f, %.2f), 朝向: %.2f", x, y, theta)

def main():
    rospy.init_node('navigation_controller_node')
    controller = NavigationController()

    # 导航到名为"kitchen"的航点
    controller.navigate_to_waypoint("kitchen")

    rospy.sleep(5)  # 等待5秒

    # 导航到指定位置
    controller.navigate_to_pose(2.0, 3.0, 1.57)  # x=2.0, y=3.0, theta=90度

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## 6. 订阅导航结果示例

### C++ 示例

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

void navigationResultCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "done") {
        ROS_INFO("导航成功完成");
    } else if(msg->data == "failure") {
        ROS_WARN("导航失败");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_result_monitor");
    ros::NodeHandle nh;

    ros::Subscriber result_sub = nh.subscribe("waypoint/navi_result",
                                            10,
                                            navigationResultCallback);

    ros::spin();
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def navigation_result_callback(msg):
    if msg.data == "done":
        rospy.loginfo("导航成功完成")
    elif msg.data == "failure":
        rospy.logwarn("导航失败")

def monitor_navigation_result():
    rospy.init_node('navigation_result_monitor', anonymous=True)

    rospy.Subscriber("waypoint/navi_result", String, navigation_result_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        monitor_navigation_result()
    except rospy.ROSInterruptException:
        pass
```

## 注意事项

1. 所有示例代码中的错误处理都是基本的实现，实际应用中应该添加更完善的错误处理机制。
2. 发布话题时应该检查是否有订阅者（使用`getNumSubscribers()`）。
3. 在实际项目中，建议添加适当的延时和重试机制。
4. 对于导航功能，建议实现超时处理机制。

## 7. 充电桩管理示例

### C++ 示例

```cpp
#include <ros/ros.h>
#include <amr_waypoint_tools/GetNumOfWaypoints.h>
#include <amr_waypoint_tools/GetWaypointByIndex.h>
#include <amr_waypoint_tools/Waypoint.h>

class ChargerManager {
public:
    ChargerManager() : nh_("~") {
        // 创建服务客户端
        num_client_ = nh_.serviceClient<amr_waypoint_tools::GetNumOfWaypoints>("/waypoint/get_num_charger");
        index_client_ = nh_.serviceClient<amr_waypoint_tools::GetWaypointByIndex>("/waypoint/get_charger_index");
        charger_pub_ = nh_.advertise<amr_waypoint_tools::Waypoint>("/waypoint/add_charger", 1);
    }

    // 获取充电桩数量
    int getChargerCount() {
        amr_waypoint_tools::GetNumOfWaypoints srv;
        if (num_client_.call(srv)) {
            ROS_INFO("充电桩数量: %d", srv.response.num);
            return srv.response.num;
        }
        return -1;
    }

    // 获取所有充电桩信息
    void getAllChargers() {
        int count = getChargerCount();
        if (count <= 0) return;

        amr_waypoint_tools::GetWaypointByIndex srv;
        for (int i = 0; i < count; i++) {
            srv.request.index = i;
            if (index_client_.call(srv)) {
                ROS_INFO("充电桩 %d - 名称: %s, 位置: (%.2f, %.2f)",
                    i,
                    srv.response.name.c_str(),
                    srv.response.pose.position.x,
                    srv.response.pose.position.y);
            }
        }
    }

    // 添加新充电桩
    void addNewCharger(const std::string& name, double x, double y) {
        amr_waypoint_tools::Waypoint charger;
        charger.frame_id = "map";
        charger.name = name;
        charger.pose.position.x = x;
        charger.pose.position.y = y;
        charger.pose.orientation.w = 1.0;

        charger_pub_.publish(charger);
        ROS_INFO("添加新充电桩: %s", name.c_str());
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient num_client_;
    ros::ServiceClient index_client_;
    ros::Publisher charger_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "charger_manager_node");

    ChargerManager manager;

    // 获取并显示所有充电桩
    manager.getAllChargers();

    // 添加新充电桩
    manager.addNewCharger("charger_1", 3.0, 4.0);

    ros::spin();
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python
import rospy
from amr_waypoint_tools.srv import GetNumOfWaypoints, GetWaypointByIndex
from amr_waypoint_tools.msg import Waypoint
from geometry_msgs.msg import Pose

class ChargerManager:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('charger_manager_node', anonymous=True)

        # 创建服务代理和发布者
        self.num_client = rospy.ServiceProxy('/waypoint/get_num_charger',
                                           GetNumOfWaypoints)
        self.index_client = rospy.ServiceProxy('/waypoint/get_charger_index',
                                             GetWaypointByIndex)
        self.charger_pub = rospy.Publisher('/waypoint/add_charger',
                                         Waypoint,
                                         queue_size=1)

    def get_charger_count(self):
        try:
            response = self.num_client()
            rospy.loginfo("充电桩数量: %d", response.num)
            return response.num
        except rospy.ServiceException as e:
            rospy.logerr("获取充电桩数量失败: %s", e)
            return -1

    def get_all_chargers(self):
        count = self.get_charger_count()
        if count <= 0:
            return

        for i in range(count):
            try:
                response = self.index_client(i)
                rospy.loginfo("充电桩 %d - 名称: %s, 位置: (%.2f, %.2f)",
                            i,
                            response.name,
                            response.pose.position.x,
                            response.pose.position.y)
            except rospy.ServiceException as e:
                rospy.logerr("获取充电桩信息失败: %s", e)

    def add_new_charger(self, name, x, y):
        charger = Waypoint()
        charger.frame_id = "map"
        charger.name = name
        charger.pose = Pose()
        charger.pose.position.x = x
        charger.pose.position.y = y
        charger.pose.orientation.w = 1.0

        self.charger_pub.publish(charger)
        rospy.loginfo("添加新充电桩: %s", name)

def main():
    manager = ChargerManager()

    # 获取并显示所有充电桩
    manager.get_all_chargers()

    # 添加新充电桩
    manager.add_new_charger("charger_1", 3.0, 4.0)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## 8. 完整的导航应用示例

### C++ 示例

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <amr_waypoint_tools/GetWaypointByName.h>

class NavigationApplication {
public:
    NavigationApplication() : nh_("~") {
        // 初始化服务客户端和发布者
        wp_client_ = nh_.serviceClient<amr_waypoint_tools::GetWaypointByName>("/waypoint/get_waypoint_name");
        navi_pub_ = nh_.advertise<std_msgs::String>("/waypoint/navi_waypoint", 1);
        result_sub_ = nh_.subscribe("/waypoint/navi_result", 10,
                                  &NavigationApplication::navigationResultCallback,
                                  this);
    }

    // 开始导航任务
    void startNavigation(const std::string& target_point) {
        current_target_ = target_point;

        // 获取目标点位置
        amr_waypoint_tools::GetWaypointByName srv;
        srv.request.name = target_point;

        if (wp_client_.call(srv)) {
            ROS_INFO("开始导航到目标点: %s", target_point.c_str());
            ROS_INFO("目标位置: (%.2f, %.2f)",
                    srv.response.pose.position.x,
                    srv.response.pose.position.y);

            // 发送导航命令
            std_msgs::String msg;
            msg.data = target_point;
            navi_pub_.publish(msg);
        } else {
            ROS_ERROR("无法获取目标点信息: %s", target_point.c_str());
        }
    }

private:
    void navigationResultCallback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == "done") {
            ROS_INFO("成功到达目标点: %s", current_target_.c_str());
            // 这里可以添加到达目标点后的操作
        } else if (msg->data == "failure") {
            ROS_WARN("导航到目标点失败: %s", current_target_.c_str());
            // 这里可以添加导航失败后的处理逻辑
        }
    }

    ros::NodeHandle nh_;
    ros::ServiceClient wp_client_;
    ros::Publisher navi_pub_;
    ros::Subscriber result_sub_;
    std::string current_target_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_application");

    NavigationApplication app;

    // 示例：执行一系列导航任务
    std::vector<std::string> waypoints = {"kitchen", "living_room", "bedroom"};

    for (const auto& wp : waypoints) {
        app.startNavigation(wp);
        ros::Duration(10.0).sleep();  // 等待导航完成
    }

    ros::spin();
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from amr_waypoint_tools.srv import GetWaypointByName

class NavigationApplication:
    def __init__(self):
        rospy.init_node('navigation_application', anonymous=True)

        # 初始化服务客户端和发布者
        self.wp_client = rospy.ServiceProxy('/waypoint/get_waypoint_name',
                                          GetWaypointByName)
        self.navi_pub = rospy.Publisher('/waypoint/navi_waypoint',
                                      String,
                                      queue_size=1)
        self.result_sub = rospy.Subscriber('/waypoint/navi_result',
                                         String,
                                         self.navigation_result_callback)
        self.current_target = ''

    def start_navigation(self, target_point):
        self.current_target = target_point

        try:
            # 获取目标点位置
            response = self.wp_client(target_point)
            rospy.loginfo("开始导航到目标点: %s", target_point)
            rospy.loginfo("目标位置: (%.2f, %.2f)",
                         response.pose.position.x,
                         response.pose.position.y)

            # 发送导航命令
            msg = String()
            msg.data = target_point
            self.navi_pub.publish(msg)

        except rospy.ServiceException as e:
            rospy.logerr("无法获取目标点信息: %s - %s", target_point, str(e))

    def navigation_result_callback(self, msg):
        if msg.data == "done":
            rospy.loginfo("成功到达目标点: %s", self.current_target)
            # 这里可以添加到达目标点后的操作
        elif msg.data == "failure":
            rospy.logwarn("导航到目标点失败: %s", self.current_target)
            # 这里可以添加导航失败后的处理逻辑

def main():
    app = NavigationApplication()

    # 示例：执行一系列导航任务
    waypoints = ["kitchen", "living_room", "bedroom"]

    for wp in waypoints:
        app.start_navigation(wp)
        rospy.sleep(10)  # 等待导航完成

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## 使用建议

1. 错误处理

- 添加超时机制
- 实现重试逻辑
- 记录详细的错误信息

2. 性能优化

- 适当设置队列大小
- 合理控制发布频率
- 及时清理不需要的订阅

3. 调试技巧

- 使用 ROS 的日志级别（DEBUG, INFO, WARN, ERROR）
- 使用 rqt_graph 查看节点关系
- 使用 rostopic echo 监控话题

4. 最佳实践

- 使用 nodelets 提高性能
- 实现优雅的启动和关闭
- 添加参数配置功能
- 使用 launch 文件组织节点
