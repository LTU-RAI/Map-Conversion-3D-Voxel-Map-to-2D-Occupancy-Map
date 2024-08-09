#pragma once

#include "HeightRangeMap.hh"
#include "Map2D.hh"
#include "datatypes.hh"

using namespace std;

class MapConverter {
public:
  Map2D map;
  MapConverter(double resolution, int slopeEstimationSize, double minimumZ,
               int minimumOccupancy);
  ~MapConverter();
  // update global map in region of the voxels given
  void updateMap(vector<voxel> vMap, vector<double> minMax);
  // get the min max values in x and y for given voxel list agins map frame.
  // Returns vector with <minX, maxX, minY, maxY, minZ, maxZ>
  vector<double> minMaxVoxel(vector<voxel> list);

private:
  int slopeEstimationSize;
  double resolution;
  int minOcc;
  double minZ;
};
