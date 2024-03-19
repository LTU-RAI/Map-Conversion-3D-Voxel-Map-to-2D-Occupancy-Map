#pragma once

#include "datatypes.hh"
#include "Map2D.hh"
#include "HeightRangeMap.hh"

using namespace std;

class MapConverter
{
public:
    Map2D map;
    MapConverter(double resolution, double minimumZ, int minimumOccupancy);
    ~MapConverter();
    //update global map in region of the voxels given
    void updateMap(vector<voxel> vMap);

private:
    double resolution;
    int minOcc;
    double minZ;

    //get the min max values in x and y for given voxel list agins map frame. Returns vector with <minX, maxX, minY, maxY>
    vector<double> minMaxVoxel(vector<voxel> list);
    
    //merge sub map with global map 
    void insertMap(Map2D newMap);
};