#include "datatypes.hh"

#include <geometry_msgs/Pose.h>
#include <mapconversion/HeightMap.h>
#include <nav_msgs/Path.h>

using namespace std;

class PathConverter {
private:
  // ROS nh
  ros::NodeHandle nh;
  // subs
  ros::Subscriber subPath, subHeight;
  // pub
  ros::Publisher pubPath;

  // Map data
  mapconversion::HeightMap hMap;
  nav_msgs::Path cPath;
  double currentResulution = 0;

  // collision area
  bool collsionShape;
  double collisionRadius;

  struct collisionPoint {
    int x, y;
    double z;
  };

  vector<collisionPoint> collisionArea;

  // path param
  int pathSmothingLength;
  double pathOffset;

public:
  PathConverter() {
    ros::NodeHandle nh_priv("~");
    collsionShape = nh_priv.param("use_collision_sphere", false);
    collisionRadius = nh_priv.param("collision_radius", 1.0);
    pathOffset = nh_priv.param("path_offset", 0.0);
    pathSmothingLength = nh_priv.param("path_smothing_length", 5);

    subHeight =
        nh.subscribe("heightMap", 1, &PathConverter::heightCallback, this);

    subPath = nh.subscribe("pathIn", 10, &PathConverter::pathCallback, this);

    pubPath = nh.advertise<nav_msgs::Path>("pathOut", 10);
    ;
  }

  ~PathConverter() {}

  void heightCallback(mapconversion::HeightMap newHeightMap) {
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

  void pathCallback(nav_msgs::Path inPath) {
    cPath = inPath;
    updatePath();
  }

  void updatePath() {
    if (collisionArea.size() == 0)
      return;
    nav_msgs::Path outPath = cPath;

    setPathHeight3D(&outPath);

    if (collsionShape)
      solveCollisionUAV(&outPath);

    pubPath.publish(outPath);
  }

  // get the hight over floor using pathOffset
  void setPathHeight3D(nav_msgs::Path *path) {
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
  void solveCollisionUAV(nav_msgs::Path *path) {
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

  point_int worldToMap(geometry_msgs::Pose position) {
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
  ros::init(argc, argv, "path_converter");

  PathConverter PC;
  ros::spin();

  return 0;
}
