
#include "MapConverter.hh"

MapConverter::MapConverter(double resolution, double minimumZ, int minimumOccupancy){
    MapConverter::resolution=resolution; 
    MapConverter::minOcc=minimumOccupancy;
    MapConverter::minZ=minimumZ;
    Map2D m(resolution);
    MapConverter::map.update(m);
}

MapConverter::~MapConverter(){
}

void MapConverter::updateMap(vector<voxel> vMap){
    if(vMap.size()==0)return;
    vector<double> minMax =MapConverter::minMaxVoxel(vMap);
    for(double mM : minMax) if(isinf(mM)) return; //If too few voxels of target resolution reserved to get the size
    //get number of cells in grid
    int xSize=(minMax[1]-minMax[0])/MapConverter::map.getResulution();
    int ySize=(minMax[3]-minMax[2])/MapConverter::map.getResulution();
    //creat local maps
    HeightRangeMap hMap(xSize,ySize);
    Map2D newMap(MapConverter::resolution,xSize,ySize,minMax[0],minMax[2]);
    for(voxel v : vMap){
        //as voxels can be larger then a map cell, this loop goes through all cell voxel occupies
        int sizeIndex=v.halfSize*2/MapConverter::map.getResulution();
        for(int x=0; x<sizeIndex;x++){
            for(int y=0; y<sizeIndex;y++){
                int posX=(v.position.x-minMax[0]-v.halfSize+resolution*x)/resolution;
                int posY=(v.position.y-minMax[2]-v.halfSize+resolution*y)/resolution;
                if(posX<0||posX>=xSize) continue; //if cell is outside local map grid
                if(posY<0||posY>=ySize) continue;
                //size of heigt range is slightly enlarged for better overlap detection
                heightRange hr={v.position.z+v.halfSize+resolution*0.01,v.position.z-v.halfSize-resolution*0.01};
                hMap.addRange(posX,posY,hr,v.occupied);
            }
        }
    }
    //remove free space that is smaller than robots is safety margin
    for(int x=0;x<xSize;x++){
        for(int y=0;y<ySize;y++){           
            hMap.removeFreeRanges(x,y,minZ);

            if(hMap.free[x][y].size()==0) continue;
            //set free space and heigth map
            newMap.set(x,y,0);
            newMap.setHeight(x,y,hMap.free[x][y].back().bottom);
            newMap.setHeightTop(x,y,hMap.free[x][y][0].top);
        }
    }
    //check if there are a neighboring occupied cells
    for(int x=0;x<xSize;x++){
        for(int y=0;y<ySize;y++){           
            //find free space with a unknown neighbour 
            if(hMap.free[x][y].size()==0) continue;
            
            for(auto d : DIRECTIONS){
                if(x+d.x>=xSize || x+d.x<0) continue;
                if(y+d.y>=ySize || y+d.y<0) continue;
                if(newMap.get(x+d.x,y+d.y)!=-1) continue;
                newMap.set(x+d.x,y+d.y,hMap.getOccupation(x+d.x,y+d.y,-1,minOcc));
            }
        }
    }
    MapConverter::insertMap(newMap);
}

vector<double> MapConverter::minMaxVoxel(vector<voxel> list){
    vector<double> minMax(4);
    minMax[0]=INFINITY;
    minMax[1]=-INFINITY;
    minMax[2]=INFINITY;
    minMax[3]=-INFINITY;
    for(voxel v : list){
        //for reliebol size estimation of new regon, only smalest voxels in the octree are used
        if(v.halfSize>resolution*.9) continue;
        minMax[0]=min(minMax[0],v.position.x);
        minMax[1]=max(minMax[1],v.position.x);
        minMax[2]=min(minMax[2],v.position.y);
        minMax[3]=max(minMax[3],v.position.y);
    }
    return minMax;
}

void MapConverter::insertMap(Map2D newMap){
    newMap.updateSlope();
    if(MapConverter::map.sizeX()==0){
        map.update(newMap);
        return;
    }
    //get size and offset of new global map 
    double newOffsetX=min(map.offsetX(),newMap.offsetX());
    double maxX=max(map.offsetX()+map.sizeX()*resolution,
                    newMap.offsetX()+newMap.sizeX()*resolution);
    int newSizeX=(maxX-newOffsetX)/resolution;

    double newOffsetY=min(map.offsetY(),newMap.offsetY());
    double maxY=max(map.offsetY()+map.sizeY()*resolution,
                    newMap.offsetY()+newMap.sizeY()*resolution);
    int newSizeY=(maxY-newOffsetY)/resolution;
    //creat new global map
    Map2D mergedMap(resolution, newSizeX, newSizeY, 
                    newOffsetX, newOffsetY);
    
    //copy old global map into new global map
    int offsetDifX=round((map.offsetX()-mergedMap.offsetX())/resolution);
    int offsetDifY=round((map.offsetY()-mergedMap.offsetY())/resolution);
    for(int x=0;x<map.sizeX();x++){
        for(int y=0;y<map.sizeY();y++){
            mergedMap.set(x+offsetDifX,y+offsetDifY,map.get(x,y));
            mergedMap.setHeight(x+offsetDifX,y+offsetDifY,map.getHeight(x,y));
            mergedMap.setHeightTop(x+offsetDifX,y+offsetDifY,map.getHeightTop(x,y));
            mergedMap.setSlope(x+offsetDifX,y+offsetDifY,map.getSlope(x,y));
        }
    }

    //copy new map into new global map
    offsetDifX=round((newMap.offsetX()-mergedMap.offsetX())/resolution);
    offsetDifY=round((newMap.offsetY()-mergedMap.offsetY())/resolution);
    for(int x=1;x<newMap.sizeX()-1;x++){
        for(int y=1;y<newMap.sizeY()-1;y++){
            mergedMap.set(x+offsetDifX,y+offsetDifY,newMap.get(x,y));
            mergedMap.setHeight(x+offsetDifX,y+offsetDifY,newMap.getHeight(x,y));
            mergedMap.setHeightTop(x+offsetDifX,y+offsetDifY,newMap.getHeightTop(x,y));
            mergedMap.setSlope(x+offsetDifX,y+offsetDifY,newMap.getSlope(x,y));
        }
    }

    map.update(mergedMap);
}
