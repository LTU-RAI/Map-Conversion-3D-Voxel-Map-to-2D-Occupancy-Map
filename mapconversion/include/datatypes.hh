#pragma once
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

struct point_int {
  int x, y;
};

struct point2D {
  double x, y;
};

struct point3D {
  double x, y, z;
};

struct voxel {
  point3D position;
  float halfSize;
  bool occupied;
};

struct heightRange {
  double top, bottom;
};
