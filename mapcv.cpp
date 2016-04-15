#include "mapcv.h"

MapCV::MapCV()
{
    Size sz;
    map=imread(M_CV_MAP );
    pyrDown(map,map,sz,cv::BORDER_DEFAULT); //1:0.5  zoom map

    double fScale = M_CV_FSCALE;            //zoom rate
    IplImage *pCar = cvLoadImage(M_CV_CAR,CV_LOAD_IMAGE_UNCHANGED);
    sz.width=pCar->width*fScale;
    sz.height=pCar->height*fScale;
    IplImage *ptmp=cvCreateImage(sz,pCar->depth,pCar->nChannels);
    cvResize(pCar,ptmp,CV_INTER_AREA);
    cvSaveImage("zoom_marke.png",ptmp);
    cvReleaseImage(&pCar);
    cvReleaseImage(&ptmp);
    car = imread("zoom_marke.png");
    car_mask = imread("zoom_marke.png",0);


    fScale = M_CV_FSCALE_DOOR;            //zoom rate
    pCar = cvLoadImage(M_CV_DOOR,CV_LOAD_IMAGE_UNCHANGED);
    sz.width=pCar->width*fScale;
    sz.height=pCar->height*fScale;
    ptmp=cvCreateImage(sz,pCar->depth,pCar->nChannels);
    cvResize(pCar,ptmp,CV_INTER_AREA);
    cvSaveImage("zoom_door.png",ptmp);
    cvReleaseImage(&pCar);
    cvReleaseImage(&ptmp);
    door = imread("zoom_door.png");
    door_mask = imread("zoom_door.png",0);
    doorNum=0;
    pdoormask = new bool[MAX_DOOR];
    memset(pdoormask,false,sizeof(bool)*MAX_DOOR);




    fScale = M_CV_FSCALE_DOOR1;            //zoom rate
    pCar = cvLoadImage(M_CV_DOOR1,CV_LOAD_IMAGE_UNCHANGED);
    sz.width=pCar->width*fScale;
    sz.height=pCar->height*fScale;
    ptmp=cvCreateImage(sz,pCar->depth,pCar->nChannels);
    cvResize(pCar,ptmp,CV_INTER_AREA);
    cvSaveImage("zoom_door1.png",ptmp);
    cvReleaseImage(&pCar);
    cvReleaseImage(&ptmp);
    door1 = imread("zoom_door1.png");
    door1_mask = imread("zoom_door1.png",0);

    map_msg.Map_matrix = cvCreateMat(2,2,CV_32FC1);
    map_msg.GPS_matrix = cvCreateMat(2,2,CV_32FC1);
    map_msg.shift_matrix = cvCreateMat(2,1,CV_32FC1);
    map_msg.C_g_m = cvCreateMat(2,2,CV_32FC1);

    car_msg.Map_matrix = cvCreateMat(2,1,CV_32FC1);
    car_msg.GPS_matrix = cvCreateMat(2,1,CV_32FC1);
    car_msg.center  = Point(car.cols/2,car.rows/2);
    car_msg.mask_center  = Point(car_mask.cols/2,car_mask.rows/2);

    selectDoor =0;
}
void MapCV::SetMapPose(float gps_x1, float gps_y1, float gps_x2, float gps_y2, float gps_x3, float gps_y3)
{
    // save GPS message in matrix
    cvmSet(map_msg.GPS_matrix,0,0,gps_x2-gps_x1);
    cvmSet(map_msg.GPS_matrix,1,0,gps_y2-gps_y1);
    cvmSet(map_msg.GPS_matrix,0,1,gps_x3-gps_x1);
    cvmSet(map_msg.GPS_matrix,1,1,gps_y3-gps_y1);

    // save map image message in matrix
    cvmSet(map_msg.Map_matrix,0,0,map.cols);
    cvmSet(map_msg.Map_matrix,1,0,0);
    cvmSet(map_msg.Map_matrix,0,1,0);
    cvmSet(map_msg.Map_matrix,1,1,map.rows);


    // get C_g_m
    CvMat *inverse = cvCreateMat(2,2,CV_32FC1);
    cvInvert(map_msg.GPS_matrix,inverse,CV_SVD);
    cvGEMM(map_msg.Map_matrix,inverse,1,NULL,0,map_msg.C_g_m);

    // get shift_matrix
    cvmSet(map_msg.shift_matrix,0,0,-gps_x1);
    cvmSet(map_msg.shift_matrix,1,0,-gps_y1);
    cvGEMM(map_msg.C_g_m,map_msg.shift_matrix,1,NULL,0,map_msg.shift_matrix);
}
void MapCV::SetCarPose(float gps_x, float gps_y, float yaw)
{

    cvmSet(car_msg.GPS_matrix,0,0,gps_x);
    cvmSet(car_msg.GPS_matrix,1,0,gps_y);

    cvGEMM(map_msg.C_g_m,car_msg.GPS_matrix,1,NULL,0,car_msg.Map_matrix);
    cvAdd(car_msg.Map_matrix,map_msg.shift_matrix,car_msg.Map_matrix);

    car_msg.map_x = int(cvmGet(car_msg.Map_matrix,0,0));
    car_msg.map_y = int(cvmGet(car_msg.Map_matrix,1,0));
    car_msg.map_angle   =   yaw;
    //qDebug()<<cvmGet(car_msg.Map_matrix,0,0)<<' '<<cvmGet(car_msg.Map_matrix,1,0);
}
void MapCV::SetDoor(int num, float gps_x, float gps_y)
{
    if(MAX_DOOR < num || num> doorNum )
    {
        std::cerr<<"error  unreasonable sequence number.\n";
        return;
    }else if(num == doorNum)
    {
        pdoormask[num]=true;
        doorNum++;
    }else
        pdoormask[num]=true;


    CvMat *gps = cvCreateMat(2,1,CV_32FC1);
    CvMat *map = cvCreateMat(2,1,CV_32FC1);

    cvmSet(gps,0,0,gps_x);
    cvmSet(gps,1,0,gps_y);

    cvGEMM(map_msg.C_g_m, gps,1,NULL,0,map);
    cvAdd(map, map_msg.shift_matrix, map);

    door_msg[num].map_x = int(cvmGet(map,0,0));
    door_msg[num].map_y = int(cvmGet(map,1,0));

}
void MapCV::DeleteDoor(int num)
{
    if( num == doorNum)
        doorNum--;
    else
        pdoormask[num] = false;
}
void MapCV::SetObstacle(int group, float x, float y)  //MySQL may be can solve it
{

}
void MapCV::ShowMap()
{
    Mat r;
    //SetMapPose(90,80,150,0,10,20);
    //SetDoor(0,90,80);
    //SetDoor(1,60,40);
    //SetDoor(2,30,150);
    //DeleteDoor(2);
    r=ShowDoor(map,selectDoor);


    SetCarPose(210,-80,20);
    imshow("Map",ShowCar(r));
   // waitKey(100);
    //SetCarPose(40,-20,20);
    //imshow("Map",ShowCar(r));
}
Mat  MapCV::ShowCar(Mat src)
{
    Mat rotMat(2,3,CV_32FC1);
    Mat car_rotatr;
    Mat car_mask_rotatr;
    rotMat = getRotationMatrix2D(car_msg.center, car_msg.map_angle,0.3);
    warpAffine(car, car_rotatr, rotMat, car.size());
    rotMat = getRotationMatrix2D(car_msg.mask_center, car_msg.map_angle,0.3);
    warpAffine(car_mask, car_mask_rotatr, rotMat, car_mask.size());

    Mat car_imageROI;
    Mat car_mask_imageROI;
    check(car_rotatr, car_mask_rotatr, &car_imageROI, &car_mask_imageROI, &car_msg.map_x,&car_msg.map_y);
    /*
    if( (car_msg.map_x-car_rotatr.cols/2) < 0 )
    {
        car_imageROI = car_rotatr( Rect(car_rotatr.cols/2-car_msg.map_x, 0, car_rotatr.cols/2+car_msg.map_x, car_rotatr.rows) );
        car_mask_imageROI = car_mask_rotatr( Rect(car_mask_rotatr.cols/2-car_msg.map_x, 0,
                                                       car_mask_rotatr.cols/2+car_msg.map_x, car_mask_rotatr.rows) );
        car_msg.map_x=0;
    }else if(  (car_msg.map_x+car_rotatr.cols/2) > map.cols)
    {
        car_imageROI = car_rotatr( Rect(0, 0, car_rotatr.cols/2-car_msg.map_x+map.cols, car_rotatr.rows) );
        car_mask_imageROI = car_mask_rotatr( Rect(0, 0, car_mask_rotatr.cols/2-car_msg.map_x+map.cols, car_mask_rotatr.rows) );
        car_msg.map_x -= car_rotatr.cols/2;
    }else
    {
        car_imageROI = car_rotatr( Rect(0, 0, car_rotatr.cols, car_rotatr.rows) );
        car_mask_imageROI = car_mask_rotatr(Rect(0, 0, car_mask_rotatr.cols, car_mask_rotatr.rows)  );
        car_msg.map_x -=car_rotatr.cols/2;
    }

    if( (car_msg.map_y-car_rotatr.rows/2) < 0 )
    {
        car_imageROI = car_imageROI( Rect(0, car_imageROI.rows/2-car_msg.map_y,
                                          car_imageROI.cols, car_imageROI.rows/2+car_msg.map_y) );
        car_mask_imageROI = car_mask_imageROI( Rect(0, car_mask_imageROI.rows/2-car_msg.map_y,
                                                      car_mask_imageROI.cols, car_mask_imageROI.rows/2+car_msg.map_y)  );
        car_msg.map_y=0;
    }else if( (car_msg.map_y+car_rotatr.rows/2) > map.rows)
    {

        car_imageROI = car_imageROI( Rect(0, 0, car_imageROI.cols, car_imageROI.rows/2-car_msg.map_y+map.rows) );
        car_mask_imageROI = car_mask_imageROI( Rect(0, 0, car_mask_imageROI.cols,
                                                    car_mask_imageROI.rows/2-car_msg.map_y+map.rows) );
        car_msg.map_y -= car_rotatr.rows/2;
    }else
    {
        car_msg.map_y -= car_rotatr.rows/2;
    }*/

    Mat map_copy;
    Mat map_copy_imageROI;
    map_copy=src.clone();
    map_copy_imageROI = map_copy(  Rect(car_msg.map_x, car_msg.map_y, car_imageROI.cols, car_imageROI.rows) );
    car_imageROI.copyTo(map_copy_imageROI,car_mask_imageROI);
    return map_copy;
}
Mat  MapCV::ShowDoor(Mat src ,int select_door)
{
    Mat door_imageROI,door_mask_imageROI;
    Mat map_copy,map_copy_imageROI;
    map_copy=src.clone();

    CvFont font;
    double hScale=0.7,vScale=0.7;
    int lineWidth =2;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);

    IplImage pimage= IplImage(map_copy);
    int x,y;
    for(int i=0; i < doorNum; i++)
    {
        if( pdoormask[i] )
        {
            x=door_msg[i].map_x;
            y=door_msg[i].map_y;
            if(  i == select_door )
            {
                check(door1, door1_mask, &door_imageROI, &door_mask_imageROI, &x, &y);
                map_copy_imageROI = map_copy(  Rect(x, y, door_imageROI.cols, door_imageROI.rows) );
                door_imageROI.copyTo(map_copy_imageROI,door_mask_imageROI);
            }else
            {
                check(door, door_mask, &door_imageROI, &door_mask_imageROI, &x, &y);
                map_copy_imageROI = map_copy(  Rect(x, y, door_imageROI.cols, door_imageROI.rows) );
                door_imageROI.copyTo(map_copy_imageROI,door_mask_imageROI);
            }
            if(y-17 <0)
            {
                std::stringstream msg;
                msg<<i;
                const char *ss = msg.str().c_str();
                cvPutText( &pimage, ss, cvPoint(x, y+door_imageROI.rows+17), &font, CV_RGB(0,0,0)  );
            }else
            {
                std::stringstream msg;
                msg<<i;
                const char *ss = msg.str().c_str();
                cvPutText( &pimage, ss, cvPoint(x, y), &font, CV_RGB(0,0,0)  );
            }
        }
    }
    return map_copy;
}
Mat  MapCV::ShowObstacle()
{
    return map;
}
bool  MapCV::check(Mat src, Mat src_mask, Mat *pdst, Mat *pdst_mask, int *posex, int *posey)
{
    Mat dst,dst_mask;
    bool result=true;

    dst = *pdst;
    dst_mask = *pdst_mask;

    if(0 > *posex)
    {
        *posex = 0;
        result = false;
    }else if( map.cols < *posex)
    {
        *posex = map.cols;
        result = false;
    }

    if(0 > *posey)
    {
        *posey = 0;
        result = false;
    }else if( map.rows < *posey)
    {
        *posey = map.rows;
        result = false;
    }

    if( (*posex-src.cols/2) < 0 )
    {
        dst = src( Rect(src.cols/2-*posex, 0, src.cols/2+*posex, src.rows) );
        dst_mask = src_mask( Rect(src_mask. cols/2-*posex, 0, src_mask.cols/2+*posex, src_mask.rows) );
        *posex=0;
    }else if(  (*posex+src.cols/2) > map.cols)
    {
        dst = src( Rect(0, 0, src.cols/2-*posex+map.cols, src.rows) );
        dst_mask = src_mask( Rect(0, 0, src_mask.cols/2-*posex+map.cols, src_mask.rows) );
        *posex -= src.cols/2;
    }else
    {
        dst = src( Rect(0, 0, src.cols, src.rows) );
        dst_mask = src_mask(Rect(0, 0, src_mask.cols, src_mask.rows)  );
        *posex -=src.cols/2;
    }

    if( (*posey-src.rows/2) < 0 )
    {
        dst = dst( Rect(0, dst.rows/2-*posey, dst.cols, dst.rows/2+*posey) );
        dst_mask = dst_mask( Rect(0, dst_mask.rows/2-*posey, dst_mask.cols, dst_mask.rows/2+*posey)  );
        *posey=0;
    }else if( (*posey+src.rows/2) > map.rows)
    {

        dst = dst( Rect(0, 0, dst.cols, dst.rows/2-*posey+map.rows) );
        dst_mask = dst_mask( Rect(0, 0, dst_mask.cols, dst_mask.rows/2-*posey+map.rows) );
        *posey -= src.rows/2;
    }else
    {
        *posey -= src.rows/2;
    }
    *pdst = dst;
    *pdst_mask = dst_mask;
    return result;
}
