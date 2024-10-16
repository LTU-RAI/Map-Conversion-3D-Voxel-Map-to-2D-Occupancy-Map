#include <cstdio>
#include <mapconversion/HeightMap.h>
#include <mapconversion/SlopeMap.h>
#include <nav_msgs/Odometry.h>
#include <ufo/map/occupancy_map.h>
#include <ufomap_msgs/UFOMapStamped.h>
// To convert between UFO and ROS
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include "MapConverter.hh"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

class MapToMap {
private:
  // ROS nh
  ros::NodeHandle nh;
  // subs
  ros::Subscriber subUfoMap;
  ros::Subscriber subOdom;

  // pub
  ros::Publisher pubMapUGV, pubMapUAV, pubMapFloor, pubMapCeiling, pubHeightMap,
      pubMapSlopeVis, pubMapSlope;
  // msg
  nav_msgs::OccupancyGrid mapMsg;
  mapconversion::HeightMap heightMsg;
  mapconversion::SlopeMap slopeMsg;

  MapConverter *MC;
  ufo::map::OccupancyMap *ufoMap;

  double resolution;
  double slopeMax;
  int slopeEstimationSize;
  double minimumZ;
  int minimumOccupancy;
  string mapFrame;
  double mapZpos;

public:
  MapToMap() {
    ros::NodeHandle nh_priv("~");
    slopeMax = nh_priv.param("max_slope_ugv", INFINITY);
    slopeEstimationSize = nh_priv.param("slope_estimation_size", 1);
    slopeEstimationSize = max(slopeEstimationSize, 1);
    minimumZ = nh_priv.param("minimum_z", 1.0);
    minimumOccupancy = nh_priv.param("minimum_occupancy", 10);
    mapFrame = nh_priv.param("map_frame", string("map"));
    mapZpos = nh_priv.param("map_position_z", 0.0);

    subUfoMap = nh.subscribe("/ufomap", 1, &MapToMap::mapCallback, this);

    pubMapUGV = nh.advertise<nav_msgs::OccupancyGrid>("/mapUGV", 5);
    pubMapUAV = nh.advertise<nav_msgs::OccupancyGrid>("/mapUAV", 5);
    pubMapFloor =
        nh.advertise<nav_msgs::OccupancyGrid>("/visualization_floor_map", 5);
    pubMapCeiling =
        nh.advertise<nav_msgs::OccupancyGrid>("/visualization_ceiling_map", 5);
    pubHeightMap = nh.advertise<mapconversion::HeightMap>("/heightMap", 5);
    pubMapSlopeVis =
        nh.advertise<nav_msgs::OccupancyGrid>("/visualization_slope_map", 5);
    pubMapSlope = nh.advertise<mapconversion::SlopeMap>("/slopeMap", 5);
    ufoMap = NULL;
  }

  ~MapToMap() {}

  void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg) {
    // Check that UFOMap type is suported
    if (msg->map.info.id != "occupancy_map") {
      ROS_FATAL("This ROS node does not support colored UFOMaps. Please turn "
                "off the color in UFOMaps settings.");
      ros::shutdown();
      return;
    }
    // Set resulution
    if (ufoMap == NULL) {
      resolution = msg->map.info.resolution;
      MC = new MapConverter(resolution, slopeEstimationSize, minimumZ,
                            minimumOccupancy);
      ufoMap = new ufo::map::OccupancyMap(resolution);
    }
    // Convert ROS message to UFOMap
    ufomap_msgs::msgToUfo(msg->map, *ufoMap);

    if (msg->map.info.bounding_volume.aabbs.size() <= 0)
      return;
    ufo::geometry::AABB newAABB;
    newAABB = ufomap_msgs::msgToUfo(msg->map.info.bounding_volume.aabbs[0]);
    // get height of complete map
    ufo::math::Vector3 max_point(newAABB.getMax());
    ufo::math::Vector3 min_point(newAABB.getMin());
    min_point.z() = (ufoMap->getKnownBBX().getMin().z());
    max_point.z() = (ufoMap->getKnownBBX().getMax().z());
    // create bounding box, x,y size is from new map, z is from complete map
    ufo::geometry::AABB aabb(min_point, max_point);
    update2Dmap(aabb);
    pub();
  }

  // Update 2D map in the region of the aabb
  void update2Dmap(ufo::geometry::AABB aabb) {
    ufo::math::Vector3 max_point(aabb.getMax());
    ufo::math::Vector3 min_point(aabb.getMin());
    vector<double> minMax(6);
    minMax[0] = min_point.x();
    minMax[1] = max_point.x();
    minMax[2] = min_point.y();
    minMax[3] = max_point.y();
    minMax[4] = min_point.z();
    minMax[5] = max_point.z();
    vector<voxel> voxelList;
    // get all free and occupide voxels in boundign box
    for (auto it = ufoMap->beginLeaves(aabb, true, true, false, false, 0),
              it_end = ufoMap->endLeaves();
         it != it_end; ++it) {
      voxel v;
      v.position.x = it.getX();
      v.position.y = it.getY();
      v.position.z = it.getZ();
      v.halfSize = it.getHalfSize();
      v.occupied = it.isOccupied();
      voxelList.push_back(v);
    }
    MC->updateMap(voxelList, minMax);
  }

  void pub() {
    mapMsg.header.stamp = ros::Time::now();
    mapMsg.header.frame_id = mapFrame;
    heightMsg.header = mapMsg.header;
    mapMsg.info.width = MC->map.sizeX();
    mapMsg.info.height = MC->map.sizeY();
    mapMsg.info.resolution = MC->map.getResulution();
    mapMsg.info.origin.position.x = MC->map.offsetX();
    mapMsg.info.origin.position.y = MC->map.offsetY();
    mapMsg.info.origin.position.z = mapZpos;
    mapMsg.info.origin.orientation.x = 0;
    mapMsg.info.origin.orientation.y = 0;
    mapMsg.info.origin.orientation.z = 0;
    mapMsg.info.origin.orientation.w = 1;
    heightMsg.info = mapMsg.info;
    slopeMsg.info = mapMsg.info;
    mapMsg.data.resize(mapMsg.info.width * mapMsg.info.height);
    heightMsg.top.resize(mapMsg.info.width * mapMsg.info.height);
    heightMsg.bottom.resize(mapMsg.info.width * mapMsg.info.height);
    slopeMsg.slope.resize(mapMsg.info.width * mapMsg.info.height);

    // pub map for UGV
    if (pubMapUGV.getNumSubscribers() != 0) {
      if (isinf(slopeMax)) {
        ROS_WARN(
            "The max slope UGV is not set; obstacles will not be included in "
            "the UGV map. \nAdd \'_max_slope_ugv:=x\' to rosrun command, or "
            "\'<param name=\"max_slope_ugv\" value=\"x\"/>\' in launch file.");
      }

      mapMsg.header.stamp = ros::Time::now();
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          mapMsg.data[index] = MC->map.get(x, y, slopeMax);
        }
      }
      pubMapUGV.publish(mapMsg);
    }

    // pub map for UAV
    if (pubMapUAV.getNumSubscribers() != 0) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          mapMsg.data[index] = MC->map.get(x, y);
        }
      }
      pubMapUAV.publish(mapMsg);
    }

    // pub rviz visualization for floor heigth map
    if (pubMapFloor.getNumSubscribers() != 0) {
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

      pubMapFloor.publish(mapMsg);
    }
    // pub rviz visualization for cieling heigth map
    if (pubMapCeiling.getNumSubscribers() != 0) {
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

      pubMapCeiling.publish(mapMsg);
    }

    // pub height map
    if (pubHeightMap.getNumSubscribers() != 0) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          heightMsg.bottom[index] = MC->map.getHeight(x, y);
          heightMsg.top[index] = MC->map.getHeightTop(x, y);
        }
      }
      pubHeightMap.publish(heightMsg);
    }

    // pub rviz visualization fro slope map
    if (pubMapSlopeVis.getNumSubscribers() != 0) {
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
      pubMapSlopeVis.publish(mapMsg);
    }

    // pub slope map
    if (pubMapSlope.getNumSubscribers() != 0) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          slopeMsg.slope[index] = MC->map.getSlope(x, y);
        }
      }
      pubMapSlope.publish(slopeMsg);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_saver");

  MapToMap mtm;

  ros::spin();

  return 0;
}
