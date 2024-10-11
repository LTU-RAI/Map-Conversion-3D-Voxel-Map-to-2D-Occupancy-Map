#include "MapConverter.hh"
#include "octomap/octomap_types.h"
#include "octomap_msgs/msg/octomap.hpp"
#include <cstddef>
#include <cstdio>
#include <mapconversion_msgs/msg/height_map.hpp>
#include <mapconversion_msgs/msg/slope_map.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <vector>

using namespace std;

class MapToMap : public rclcpp::Node {
private:
  // subs
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subOctMap;

  // pub
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubMapUGV,
      pubMapUAV, pubMapFloor, pubMapCeiling, pubMapSlopeVis;
  rclcpp::Publisher<mapconversion_msgs::msg::HeightMap>::SharedPtr pubHeightMap;
  rclcpp::Publisher<mapconversion_msgs::msg::SlopeMap>::SharedPtr pubMapSlope;
  // msg
  nav_msgs::msg::OccupancyGrid mapMsg;
  mapconversion_msgs::msg::HeightMap heightMsg;
  mapconversion_msgs::msg::SlopeMap slopeMsg;

  MapConverter *MC;
  octomap::OcTree *OcMap;

  double resolution;
  double slopeMax;
  int slopeEstimationSize;
  double minimumZ;
  int minimumOccupancy;
  string mapFrame;
  double mapZpos;

public:
  MapToMap() : Node("map_conversion") {
    slopeMax = this->declare_parameter("max_slope_ugv", INFINITY);
    slopeEstimationSize = this->declare_parameter("slope_estimation_size", 1);
    slopeEstimationSize = max(slopeEstimationSize, 1);
    minimumZ = this->declare_parameter("minimum_z", 1.0);
    minimumOccupancy = this->declare_parameter("minimum_occupancy", 10);
    mapFrame = this->declare_parameter("map_frame", string("map"));
    mapZpos = this->declare_parameter("map_position_z", 0.0);

    subOctMap = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap", 1,
        std::bind(&MapToMap::mapCallback, this, std::placeholders::_1));

    pubMapUGV =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapUGV", 5);
    pubMapUAV =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapUAV", 5);
    pubMapFloor = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/visualization_floor_map", 5);
    pubMapCeiling = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/visualization_ceiling_map", 5);
    pubHeightMap = this->create_publisher<mapconversion_msgs::msg::HeightMap>(
        "/heightMap", 5);
    pubMapSlopeVis = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/visualization_slope_map", 5);
    pubMapSlope = this->create_publisher<mapconversion_msgs::msg::SlopeMap>(
        "/slopeMap", 5);
    OcMap = NULL;
    MC = NULL;
  }

  ~MapToMap() {}

  void mapCallback(octomap_msgs::msg::Octomap msg) {
    // Set resulution
    if (MC == NULL) {
      resolution = msg.resolution;
      MC = new MapConverter(resolution, slopeEstimationSize, minimumZ,
                            minimumOccupancy);
    }
    // Convert ROS message to Octomap
    octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(msg);
    octomap::OcTree *newOcMap = dynamic_cast<octomap::OcTree *>(tree);
    if (!(newOcMap))
      return;

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;

    newOcMap->getMetricMin(min_x, min_y, min_z);
    newOcMap->getMetricMax(max_x, max_y, max_z);
    vector<double> minMax(6);
    if (OcMap == NULL) {
      minMax[0] = min_x;
      minMax[1] = max_x;
      minMax[2] = min_y;
      minMax[3] = max_y;
      OcMap = newOcMap;
    } else {
      computeBoundingBox(minMax, newOcMap, OcMap);
      delete OcMap;
      OcMap = newOcMap;
    }
    minMax[4] = min_z;
    minMax[5] = max_z;

    update2Dmap(minMax);
    pub();
  }

  void computeBoundingBox(vector<double> &minMax, octomap::OcTree *tree1,
                          octomap::OcTree *tree2) {
    // Variables to hold min and max coordinates
    double min_x = INFINITY;
    double min_y = INFINITY;
    double min_z = INFINITY;
    double max_x = -INFINITY;
    double max_y = -INFINITY;
    double max_z = -INFINITY;

    // Iterate over all leaf nodes in tree1
    for (octomap::OcTree::leaf_iterator it = tree1->begin_leafs(),
                                        end = tree1->end_leafs();
         it != end; ++it) {
      octomap::OcTreeKey key = it.getKey();
      octomap::OcTreeNode *node2 = tree2->search(key);
      if (node2 != nullptr) {
        bool occupied1 = tree1->isNodeOccupied(*it);
        bool occupied2 = tree2->isNodeOccupied(node2);
        if (occupied1 != occupied2) {
          // Node occupancy has changed
          double x = it.getX();
          double y = it.getY();
          double z = it.getZ();
          updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z);
        }
      } else {
        // Node not found in tree2, so it was removed
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z);
      }
    }

    minMax[0] = min_x;
    minMax[1] = max_x;
    minMax[2] = min_y;
    minMax[3] = max_y;
  }

  void updateBoundingBox(double x, double y, double z, double &min_x,
                         double &min_y, double &min_z, double &max_x,
                         double &max_y, double &max_z) {
    if (x < min_x)
      min_x = x;
    if (y < min_y)
      min_y = y;
    if (z < min_z)
      min_z = z;
    if (x > max_x)
      max_x = x;
    if (y > max_y)
      max_y = y;
    if (z > max_z)
      max_z = z;
  }

  // Update 2D map in the region of the aabb
  void update2Dmap(vector<double> minMax) {
    for (double mM : minMax)
      if (isinf(mM))
        return;

    vector<voxel> voxelList;
    // get all free and occupide voxels in boundign box
    octomap::point3d minPoint(minMax[0], minMax[2], minMax[4]);
    octomap::point3d maxPoint(minMax[1], minMax[3], minMax[5]);
    for (auto it = OcMap->begin_leafs_bbx(minPoint, maxPoint),
              it_end = OcMap->end_leafs_bbx();
         it != it_end; ++it) {
      voxel v;
      v.position.x = it.getX();
      v.position.y = it.getY();
      v.position.z = it.getZ();
      v.halfSize = it.getSize() / 2;
      v.occupied = OcMap->isNodeOccupied(*it);
      voxelList.push_back(v);
    }
    MC->updateMap(voxelList, minMax);
  }

  void pub() {
    mapMsg.header.stamp = this->now();
    mapMsg.header.frame_id = mapFrame;
    heightMsg.header = mapMsg.header;
    mapMsg.info.width = MC->map.sizeX();
    mapMsg.info.height = MC->map.sizeY();
    mapMsg.info.resolution = MC->map.getResulution();
    mapMsg.info.origin.position.x = MC->map.offsetX();
    mapMsg.info.origin.position.y = MC->map.offsetY();
    mapMsg.info.origin.position.z = mapZpos;
    heightMsg.info = mapMsg.info;
    slopeMsg.info = mapMsg.info;
    mapMsg.data.resize(mapMsg.info.width * mapMsg.info.height);
    heightMsg.top.resize(mapMsg.info.width * mapMsg.info.height);
    heightMsg.bottom.resize(mapMsg.info.width * mapMsg.info.height);
    slopeMsg.slope.resize(mapMsg.info.width * mapMsg.info.height);

    // pub map for UGV
    if (pubMapUGV->get_subscription_count() != 0) {
      if (isinf(slopeMax)) {
        RCLCPP_WARN(
            this->get_logger(),
            "The max slope UGV is not set; obstacles will not be included in "
            "the UGV map. \nAdd \'_max_slope_ugv:=x\' to rosrun command, or "
            "\'<param name=\"max_slope_ugv\" value=\"x\"/>\' in launch file.");
      }

      mapMsg.header.stamp = this->now();
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          mapMsg.data[index] = MC->map.get(x, y, slopeMax);
        }
      }
      pubMapUGV->publish(mapMsg);
    }

    // pub map for UAV
    if (pubMapUAV->get_subscription_count() != 0) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          mapMsg.data[index] = MC->map.get(x, y);
        }
      }
      pubMapUAV->publish(mapMsg);
    }

    // pub rviz visualization for floor heigth map
    if (pubMapFloor->get_subscription_count() != 0) {
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0)
        return;
      double vMax = NAN, vMin = NAN;
      // find max min for normalization
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeight(x, y);
          if (isnan(v))
            continue;
          if (isnan(vMax))
            vMax = v;
          else
            vMax = max(vMax, v);
          if (isnan(vMin))
            vMin = v;
          else
            vMin = min(vMin, v);
        }
      }

      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();
          double value = MC->map.getHeight(x, y);
          if (!isnan(value)) {
            value = (value - vMin) / (vMax - vMin) * 199;
            if (value > 99)
              value -= 199; // To get the full color range in the rviz cost map
          }
          mapMsg.data[index] = value;
        }
      }

      pubMapFloor->publish(mapMsg);
    }
    // pub rviz visualization for cieling heigth map
    if (pubMapCeiling->get_subscription_count() != 0) {
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0)
        return;
      double vMax = NAN, vMin = NAN;
      // find max min for normalization
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeightTop(x, y);
          if (isnan(v))
            continue;
          if (isnan(vMax))
            vMax = v;
          else
            vMax = max(vMax, v);
          if (isnan(vMin))
            vMin = v;
          else
            vMin = min(vMin, v);
        }
      }

      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();
          double value = MC->map.getHeightTop(x, y);
          if (!isnan(value)) {
            value = (value - vMin) / (vMax - vMin) * 199;
            if (value > 99)
              value -= 199; // To get the full color range in the rviz cost map
          }
          mapMsg.data[index] = value;
        }
      }

      pubMapCeiling->publish(mapMsg);
    }

    // pub height map
    if (pubHeightMap->get_subscription_count() != 0) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          heightMsg.bottom[index] = MC->map.getHeight(x, y);
          heightMsg.top[index] = MC->map.getHeightTop(x, y);
        }
      }
      pubHeightMap->publish(heightMsg);
    }

    // pub rviz visualization fro slope map
    if (pubMapSlopeVis->get_subscription_count() != 0) {
      double vMax = slopeMax * 2, vMin = 0;

      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();
          double value = min(MC->map.getSlope(x, y), slopeMax * 2);
          if (!isnan(value)) {
            value = (value - vMin) / (vMax - vMin) * 198 + 1;
            if (value > 99)
              value -= 199; // soooo basically, rviz costmap are weird...
          }
          mapMsg.data[index] = value;
        }
      }
      pubMapSlopeVis->publish(mapMsg);
    }

    // pub slope map
    if (pubMapSlope->get_subscription_count() != 0) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          slopeMsg.slope[index] = MC->map.getSlope(x, y);
        }
      }
      pubMapSlope->publish(slopeMsg);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto Node = make_shared<MapToMap>();
  rclcpp::spin(Node);
  rclcpp::shutdown();

  return 0;
}
