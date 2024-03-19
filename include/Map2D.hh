#pragma once
#include "datatypes.hh"

class Map2D{
    public:
        Map2D(double resolution=0, int width=0, int height=0, double offsetX=0, double offsetY=0){
            Map2D::resolution=resolution;
            setSize(width,height);
            setOffset(offsetX,offsetY);
        }

        //get all occupancy data 
        vector<int> get(){
            return data;
        }

        //get all occupancy data filtered through slope threshold
        vector<int> get(double slope){
            vector<int> newData(data.size());
            for(int i=0;i<data.size();i++){
                if(!isnan(slopeData[i]) && slopeData[i]>slope){
                    newData[i]=100;
                    continue;
                }else{
                    newData[i]=data[i];
                }
            }
            return newData;
        }

        //get occupancy data at x y
        int get(int x, int y){
            int index=x+y*sizeX();
            if(index<0||index>=data.size()) return -1;
            return data[index];
        }

        //get occupancy data at x y filtered through slope threshold
        int get(int x, int y, double maxSlope){
            int index=x+y*sizeX();
            if(index<0||index>=data.size()) return -1;
            if(getSlope(x,y)>maxSlope) return 100;
            return data[index];
        }

        //get all floor data 
        vector<double> getHeight(){
            return heightData;
        }

        //get floor data at x y 
        double getHeight(int x, int y){
            int index=x+y*sizeX();
            if(index<0||index>=heightData.size()) return NAN;
            return heightData[index];
        }

        //get all ceiling data 
        vector<double> getHeightTop(){
            return heightDataTop;
        }

        //get ceiling data at x y 
        double getHeightTop(int x, int y){
            int index=x+y*sizeX();
            if(index<0||index>=heightDataTop.size()) return NAN;
            return heightDataTop[index];
        }

        //get all slope data 
        vector<double> getSlope(){
            return slopeData;
        }

        //get slope data at x y
        double getSlope(int x, int y){
            int index=x+y*sizeX();
            if(index<0||index>=slopeData.size()) return 0.0;
            return slopeData[index];
        }

        //get math width
        int sizeX(){
            return mapSizeX;
        }

        //get map hight 
        int sizeY(){
            return mapSizeY;
        }

        //get x offset from frame origin 
        double offsetX(){
            return mapOffsetX;
        }
        //get y offset from frame origin 
        double offsetY(){
            return mapOffsetY;
        }

        double getResulution(){
            return resolution;
        }

        //set occupancy value at x y 
        void set(int x, int y, int value){
            int index=x+y*sizeX();
            if(index<0||index>=data.size()) return;
            data[index]=value;
        }

        //set floor heigt value at x y 
        void setHeight(int x, int y, double value){
            int index=x+y*sizeX();
            if(index<0||index>=heightData.size()) return;
            heightData[index]=value;
        }

        //set ceiling heigt value at x y 
        void setHeightTop(int x, int y, double value){
            int index=x+y*sizeX();
            if(index<0||index>=heightDataTop.size()) return;
            heightDataTop[index]=value;
        }

        //set slope value at x y 
        void setSlope(int x, int y, double value){
            int index=x+y*sizeX();
            if(index<0||index>=slopeData.size()) return;
            slopeData[index]=value;
        }

        //Updata size and init map data of map
        void setSize(int width, int height){
            mapSizeX=width;
            mapSizeY=height;
            data.resize(width*height);
            heightData.resize(width*height);
            heightDataTop.resize(width*height);
            slopeData.resize(width*height);
            for(int i=0;i<data.size();i++){
                data[i]=-1;
                heightData[i]=NAN;
                heightDataTop[i]=NAN;
                slopeData[i]=NAN;

            }
        }
        
        void setOffset(double offsetX, double offsetY){
            mapOffsetX=offsetX;
            mapOffsetY=offsetY;
        }

        //copy new map int current map 
        void update(Map2D newMap){
            mapOffsetX=newMap.mapOffsetX;
            mapOffsetY=newMap.mapOffsetY;
            mapSizeX=newMap.mapSizeX;
            mapSizeY=newMap.mapSizeY;
            resolution=newMap.resolution;
            data=newMap.data;
            heightData=newMap.heightData;
            heightDataTop=newMap.heightDataTop;
            slopeData=newMap.slopeData;
        }

        //update slope in region of map, update complete map if no region is given
        void updateSlope(int minX=0,int maxX=-1,int minY=0,int maxY=-1){
            if(maxX==-1) maxX=sizeX();
            if(maxY==-1) maxY=sizeY();
                
            for(int x=minX+1;x<maxX-1;x++){
                for(int y=minY+1;y<maxY-1;y++){
                    if(isnan(getHeight(x,y))) continue; //do not calculate slope for empty cells 
                    int areaSize=1;
                    vector<point3D> points;
                    double lowPoint=INFINITY;
                    double highPoiht=0;
                    for(int dx=-areaSize;dx<=areaSize;dx++){
                        for(int dy=-areaSize;dy<=areaSize;dy++){
                            if(isnan(getHeight(x+dx,y+dy))) continue;
                            point3D p={double(dx),double(dy),getHeight(x+dx,y+dy)};
                            points.push_back(p);
                            if(dx==0 && dy==0)
                                highPoiht=p.z;
                            else
                                lowPoint=min(lowPoint,p.z);
                        }
                    }
                    if(points.size()<3) continue; // to slove MS problem at least 3 points is needed 
                    setSlope(x,y,getSlopeOfPoints(points));
                }
            }
        }
    private:
        vector<int> data;
        vector<double> heightData;
        vector<double> heightDataTop;
        vector<double> slopeData;
        int mapSizeX,mapSizeY;
        double mapOffsetX, mapOffsetY;
        double resolution;

        //fits to palne and calculates slope of plane and return maximum slope 
        double getSlopeOfPoints(vector<point3D> points) {
            int n = points.size();
            double a,b,c;
            // Construct matrix A and vector B
            MatrixXd A(n, 3);
            VectorXd B(n);
            for(int i = 0; i < n; ++i) {
                A(i, 0) = points[i].x; 
                A(i, 1) = points[i].y;
                A(i, 2) = 1;          
                B(i) = points[i].z;  
            }

            // Solve for x in Ax = B using the normal equation (A^T * A) * x = A^T * B
            VectorXd x = (A.transpose() * A).ldlt().solve(A.transpose() * B);

            // Plane equation coefficients
            a = x(0);
            b = x(1);
            c = x(2);
            double maxSlope = sqrt(a*a + b*b);
            return maxSlope;
        }
};