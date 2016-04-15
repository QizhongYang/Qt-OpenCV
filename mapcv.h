#ifndef MAPCV_H
#define MAPCV_H

#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#define M_CV_MAP     "ground.jpg"
#define M_CV_CAR     "marke.png"
#define M_CV_DOOR   "door.png"
#define M_CV_DOOR1   "door1.png"
#define M_CV_FSCALE       0.2
#define M_CV_FSCALE_DOOR       0.1
#define M_CV_FSCALE_DOOR1       0.1
#define MAX_DOOR          20
typedef struct
{
    CvMat *GPS_matrix;
    CvMat *Map_matrix;
    int     map_x,map_y;
    float   map_angle;
    Point  center;
    Point  mask_center;
}STR_CAR_MSG;

typedef struct
{
    int   map_x,map_y;
}STR_DOOR_MSG;

typedef struct
{
    CvMat *GPS_matrix;
    CvMat *Map_matrix;
    CvMat *shift_matrix;
    CvMat *C_g_m;
}STR_MAP_MSG;


class MapCV
{
public:
    MapCV();
    void SetMapPose(float gps_x1, float gps_y1, float gps_x2, float gps_y2, float gps_x3, float gps_y3);
    void SetCarPose(float gps_x, float gps_y, float yaw);
    void SetObstacle(int group, float x, float y); //It can be solved by MySQL;
    void SetDoor(int num, float gps_x, float gps_y);
    void DeleteDoor(int num);
    void ShowMap();
    int   selectDoor;
private:
    Mat  ShowCar(Mat src);
    Mat  ShowDoor(Mat src, int select_door);
    Mat  ShowObstacle();
    bool  check(Mat src, Mat src_mask, Mat *pdst, Mat *pdst_mask, int *posex, int *posey);

private:
    Mat     map;
    Mat     car,car_mask;
    Mat     door,door1,door_mask,door1_mask;
    int      doorNum;    
    bool    *pdoormask;
    STR_MAP_MSG  map_msg;
    STR_CAR_MSG car_msg;
    STR_DOOR_MSG door_msg[MAX_DOOR];
};

#endif // MAPCV_H
