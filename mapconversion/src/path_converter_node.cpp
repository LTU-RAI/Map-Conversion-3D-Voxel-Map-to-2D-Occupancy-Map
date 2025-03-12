#include "datatypes.hh"
#include <geometry_msgs/msg/pose.hpp>
#include <mapconversion_msgs/msg/height_map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std;

class PathConverter : public rclcpp::Node {
private:
  // subs
  rclcpp::Subscription<mapconversion_msgs::msg::HeightMap>::SharedPtr subHeight;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
  // pub
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

  // Map data
  mapconversion_msgs::msg::HeightMap hMap;
  nav_msgs::msg::Path cPath;
  double currentResulution = 0;

  // collision area
  bool collsionShape;
  double collisionRadius;

  // QoS
  bool sub_qos_reliable;
  bool sub_qos_transient_local;
  bool pub_qos_reliable;
  bool pub_qos_transient_local;

  struct collisionPoint {
    int x, y;
    double z;
  };

  vector<collisionPoint> collisionArea;

  // path param
  int pathSmothingLength;
  double pathOffset;

public:
  PathConverter() : Node("path_converter") {
    collsionShape = this->declare_parameter("use_collision_sphere", false);
    collisionRadius = this->declare_parameter("collision_radius", 1.0);
    pathOffset = this->declare_parameter("path_offset", 0.0);
    pathSmothingLength = this->declare_parameter("path_smothing_length", 5);
    sub_qos_reliable = this->declare_parameter("subscriber_qos_reliable", true);
    pub_qos_reliable = this->declare_parameter("publisher_qos_reliable", true);
    sub_qos_transient_local =
        this->declare_parameter("subscriber_qos_transient_local", false);
    pub_qos_transient_local =
        this->declare_parameter("publisher_qos_transient_local", false);

    // QoS profiles
    rclcpp::QoS sub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(5));
    if (sub_qos_reliable)
      sub_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    else
      sub_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    if (sub_qos_transient_local)
      sub_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    else
      sub_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    rclcpp::QoS pub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(5));
    if (pub_qos_reliable)
      pub_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    else
      pub_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    if (pub_qos_transient_local)
      pub_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    else
      pub_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    subHeight = this->create_subscription<mapconversion_msgs::msg::HeightMap>(
        "heightMap", sub_qos_profile,
        std::bind(&PathConverter::heightCallback, this, std::placeholders::_1));

    subPath = this->create_subscription<nav_msgs::msg::Path>(
        "pathIn", sub_qos_profile,
        std::bind(&PathConverter::pathCallback, this, std::placeholders::_1));

    pubPath =
        this->create_publisher<nav_msgs::msg::Path>("pathOut", pub_qos_profile);
    ;
  }

  ~PathConverter() {}

  void heightCallback(mapconversion_msgs::msg::HeightMap newHeightMap) {
    hMap = newHeightMap;
    updateCollisionArea(newHeightMap.info.resolution);
    updatePath();
  }

  void updateCollisionArea(double newResulution) {
    if (newResulution == currentResulution)
      return;
    currentResulution = newResulution;
    if (currentResulution <= 0.0)
      return;

    if (collsionShape) {
      generateCollisionSphere();
    } else {
      collisionArea.clear();
      // Set collision to a point at the robot pos
      collisionArea.push_back({0, 0, 0.0});
    }
  }

  // Generate collision shape as half sphere that will be mirrored around xy
  // plane
  void generateCollisionSphere() {
    int radiusSize = int(ceil(collisionRadius / currentResulution));
    collisionArea.clear();
    for (int x = -radiusSize; x <= radiusSize; x++) {
      for (int y = -radiusSize; y <= radiusSize; y++) {
        if (sqrt(x * x + y * y) > radiusSize)
          continue;
        double rX = x * currentResulution;
        double rY = y * currentResulution;
        double rZ = sqrt(collisionRadius * collisionRadius - rX * rX - rY * rY);
        if (isnan(rZ))
          continue;
        collisionPoint p = {x, y, rZ};
        collisionArea.push_back(p);
      }
    }
  }

  void pathCallback(nav_msgs::msg::Path inPath) {
    cPath = inPath;
    updatePath();
  }

  void updatePath() {
    if (collisionArea.size() == 0)
      return;
    nav_msgs::msg::Path outPath = cPath;

    setPathHeight3D(&outPath);

    if (collsionShape)
      solveCollisionUAV(&outPath);

    pubPath->publish(outPath);
  }

  // get the hight over floor using pathOffset
  void setPathHeight3D(nav_msgs::msg::Path *path) {
    vector<double> heightList;
    for (int i = 0; i < pathSmothingLength && i < path->poses.size(); i++) {
      auto p = path->poses[i];
      point_int point = worldToMap(p.pose);
      double height = getHeight(point.x, point.y);
      if (isnan(height))
        continue;
      heightList.insert(heightList.begin(), height);
    }
    for (int i = 0; i < path->poses.size(); i++) {
      if (i + pathSmothingLength < path->poses.size()) {
        auto p = path->poses[i + pathSmothingLength];
        point_int point = worldToMap(p.pose);
        double height = getHeight(point.x, point.y);
        if (isnan(height))
          continue;
        heightList.insert(heightList.begin(), height);
        if (heightList.size() > pathSmothingLength * 2)
          heightList.pop_back();
      }
      double pathHeight = -INFINITY;
      for (double h : heightList)
        pathHeight = max(pathHeight, h);
      if (isinf(pathHeight))
        continue;
      path->poses[i].pose.position.z = pathHeight + pathOffset;
    }
  }

  // move path up or down to avoid collisions for a UAV
  void solveCollisionUAV(nav_msgs::msg::Path *path) {
    for (int i = 0; i < path->poses.size(); i++) {
      auto *pose = &path->poses[i];
      point_int point = worldToMap(pose->pose);

      // Check and solve collisions floor
      double maxHeight = pose->pose.position.z;
      for (auto ap : collisionArea) {
        double h = getHeight(point.x + ap.x, point.y + ap.y);
        if (isnan(h))
          continue;
        h += ap.z;
        if (maxHeight < h)
          maxHeight = h;
      }

      // Check and solve collisions ceiling
      double minHeight = INFINITY;
      for (auto ap : collisionArea) {
        double h = getHeightTop(point.x + ap.x, point.y + ap.y);
        if (isnan(h))
          continue;
        h -= ap.z;
        if (minHeight > h)
          minHeight = h;
      }
      // if(isinf(minHeight)) continue;
      pose->pose.position.z = min(maxHeight, minHeight);
    }
  }

  point_int worldToMap(geometry_msgs::msg::Pose position) {
    double rez = hMap.info.resolution;
    return {int((position.position.x - hMap.info.origin.position.x) / rez),
            int((position.position.y - hMap.info.origin.position.y) / rez)};
  }

  double getHeight(int x, int y) {
    int index = x + y * hMap.info.width;
    if (index < 0 || index >= hMap.bottom.size())
      return NAN;
    double value = hMap.bottom[index];
    return value;
  }

  double getHeightTop(int x, int y) {
    int index = x + y * hMap.info.width;
    if (index < 0 || index >= hMap.top.size())
      return NAN;
    double value = hMap.top[index];
    return value;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PathConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
