#pragma once
#include "datatypes.hh"

class HeightRangeMap {
public:
  vector<vector<vector<heightRange>>> free;
  vector<vector<vector<heightRange>>> occupied;
  vector<vector<point2D>> posMap;

  HeightRangeMap(int width = 0, int height = 0) { setSize(width, height); }

  // update size and init height map
  void setSize(int width, int height) {
    free.resize(width);
    occupied.resize(width);
    posMap.resize(width);
    for (int i = 0; i < width; i++) {
      free[i].resize(height);
      occupied[i].resize(height);
      posMap[i].resize(height);
    }
  }

  // add occupied and free ranges, ranges are merged with existing ranges if
  // they overlap
  void addRange(int x, int y, heightRange hr, bool isOccupied, double posX,
                double posY) {
    point2D p = {posX, posY};
    posMap[x][y] = p;
    if (isOccupied)
      inserthghtRange(hr, &occupied[x][y]);
    else
      inserthghtRange(hr, &free[x][y]);
  }

  // remove all ranges that are shorter than minSize at x y position
  void removeFreeRanges(int x, int y, double minSize) {
    for (int i = 0; i < free[x][y].size(); i++) {
      if (free[x][y][i].top - free[x][y][i].bottom > minSize)
        continue;
      free[x][y].erase(free[x][y].begin() + i);
      i--;
    }
  }

  // returns the occupation level of a cell compared to a free heightRange in
  // range 0-100
  int getOccupation(int x, int y, int index, int minOcc) {
    int occupation = -1;
    for (auto d : DIRECTIONS) {
      if (x + d.x < 0 || x + d.x >= free.size())
        continue;
      if (y + d.y < 0 || y + d.y >= free.back().size())
        continue;
      occupation =
          max(occupation, getOccupation(x, y, index, x + d.x, y + d.y, minOcc));
    }
    return occupation;
  }

  // returns the occupation level of a cell compared to a free heightRange in
  // range 0-100
  int getOccupation(int x1, int y1, int index, int x2, int y2, int minOcc) {
    if (index < 0)
      index = free[x2][y2].size() + index;
    if (index < 0 || index >= free[x2][y2].size())
      return -1;
    heightRange freeHr = free[x2][y2][index];
    vector<heightRange> occupiedHr = occupied[x1][y1];

    return getOccupation(freeHr, occupiedHr, minOcc);
  }

  // returns the occupation level of a cell compared to a free heightRange in
  // range 0-100
  int getOccupation(heightRange freeHr, vector<heightRange> occupiedHr,
                    int minOcc) {
    double overlapDistans = 0;
    for (auto oHr : occupiedHr) {
      heightRange compare = {min(freeHr.top, oHr.top),
                             max(freeHr.bottom, oHr.bottom)};
      if (compare.top - compare.bottom < 0)
        continue;
      overlapDistans += compare.top - compare.bottom;
    }
    int occupation =
        int(overlapDistans / (freeHr.top - freeHr.bottom) * 100) * 2;
    if (occupation < minOcc)
      return -1;
    return min(occupation, 100);
  }

private:
  // Inserts a heightRange in a list, of the highRanges overlap in the list,
  // they are merged
  void inserthghtRange(heightRange hr, vector<heightRange> *list) {
    int overlapIndex = -1;
    for (int i = 0; i < list->size(); i++) {
      if (hr.top < list->at(i).bottom)
        continue;
      if (hr.bottom > list->at(i).top)
        continue;
      overlapIndex = i;
      break;
    }
    if (overlapIndex != -1) {
      heightRange hrNew;
      heightRange hrOld = list->at(overlapIndex);
      list->erase(list->begin() + overlapIndex);
      hrNew.top = max(hrOld.top, hr.top);
      hrNew.bottom = min(hrOld.bottom, hr.bottom);
      inserthghtRange(hrNew, list);
      return;
    }
    for (int i = 0; i < list->size(); i++) {
      if (hr.bottom <= list->at(i).top)
        continue;
      list->insert(list->begin() + i, hr);
      return;
    }
    list->push_back(hr);
  }
};
