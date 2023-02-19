#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

const double PI = 3.1415926;

double scanVoxelSize = 0.05; 
double decayTime = 2.0;
double noDecayDis = 4.0; 
double clearingDis = 8.0;
bool clearingCloud = false;
bool useSorting = true;
double quantileZ = 0.25;
bool considerDrop = false; 
bool limitGroundLift = false;
double maxGroundLift = 0.15;
bool clearDyObs = false;
double minDyObsDis = 0.3;
double minDyObsAngle = 0; 
double minDyObsRelZ = -0.5;
double minDyObsVFOV = -16.0;
double maxDyObsVFOV = 16.0;
int minDyObsPointNum = 1;
bool noDataObstacle = false;
int noDataBlockSkipNum = 0;
int minBlockPointNum = 10;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2;

// terrain voxel parameters
float terrainVoxelSize = 1.0;
int terrainVoxelShiftX = 0; 
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

// planar voxel parameters
float planarVoxelSize = 0.2;
const int planarVoxelWidth = 51;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;

pcl::PointCloud<pcl::PointXYZI>::Ptr
        laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
        laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
        laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
        terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
        terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum]; 

int terrainVoxelUpdateNum[terrainVoxelNum] = {0};
float terrainVoxelUpdateTime[terrainVoxelNum] = {0};
float planarVoxelElev[planarVoxelNum] = {0};
int planarVoxelEdge[planarVoxelNum] = {0};
int planarVoxelDyObs[planarVoxelNum] = {0};
vector<float> planarPointElev[planarVoxelNum];

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;
int noDataInited = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

// state estimation callback function
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom) {
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
            .getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;

    sinVehicleRoll = sin(vehicleRoll);
    cosVehicleRoll = cos(vehicleRoll);
    sinVehiclePitch = sin(vehiclePitch);
    cosVehiclePitch = cos(vehiclePitch);
    sinVehicleYaw = sin(vehicleYaw);
    cosVehicleYaw = cos(vehicleYaw);

    if (noDataInited == 0) {
        vehicleXRec = vehicleX;
        vehicleYRec = vehicleY;
        noDataInited = 1;
    }
    if (noDataInited == 1) {
        float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                         (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
        if (dis >= noDecayDis)
            noDataInited = 2;
    }
}

tf::TransformListener *tf_listener_;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2) {
    laserCloudTime = laserCloud2->header.stamp.toSec();

    if (!systemInited) {
        systemInitTime = laserCloudTime;
        systemInited = true;
    }

    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
        point = laserCloud->points[i];

        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                         (pointY - vehicleY) * (pointY - vehicleY));
        if (pointZ - vehicleZ > minRelZ - disRatioZ * dis && 
            pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
            dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) { 
            point.x = pointX;
            point.y = pointY;
            point.z = pointZ;
            point.intensity = laserCloudTime - systemInitTime;
            laserCloudCrop->push_back(point);
        }
    }

    newlaserCloud = true;
}

// joystick callback function
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {
    if (joy->buttons[5] > 0.5) {
        noDataInited = 0;
        clearingCloud = true;
    }
}

// cloud clearing callback function
void clearingHandler(const std_msgs::Float32::ConstPtr &dis) {
    noDataInited = 0;
    clearingDis = dis->data;
    clearingCloud = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "terrainAnalysis");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    nhPrivate.getParam("scanVoxelSize", scanVoxelSize);
    nhPrivate.getParam("decayTime", decayTime);
    nhPrivate.getParam("noDecayDis", noDecayDis);
    nhPrivate.getParam("clearingDis", clearingDis);
    nhPrivate.getParam("useSorting", useSorting);
    nhPrivate.getParam("quantileZ", quantileZ);
    nhPrivate.getParam("considerDrop", considerDrop);
    nhPrivate.getParam("limitGroundLift", limitGroundLift);
    nhPrivate.getParam("maxGroundLift", maxGroundLift);
    nhPrivate.getParam("clearDyObs", clearDyObs);
    nhPrivate.getParam("minDyObsDis", minDyObsDis);
    nhPrivate.getParam("minDyObsAngle", minDyObsAngle);
    nhPrivate.getParam("minDyObsRelZ", minDyObsRelZ);
    nhPrivate.getParam("minDyObsVFOV", minDyObsVFOV);
    nhPrivate.getParam("maxDyObsVFOV", maxDyObsVFOV);
    nhPrivate.getParam("minDyObsPointNum", minDyObsPointNum);
    nhPrivate.getParam("noDataObstacle", noDataObstacle);
    nhPrivate.getParam("noDataBlockSkipNum", noDataBlockSkipNum);
    nhPrivate.getParam("minBlockPointNum", minBlockPointNum);
    nhPrivate.getParam("vehicleHeight", vehicleHeight);
    nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);
    nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);
    nhPrivate.getParam("minRelZ", minRelZ);
    nhPrivate.getParam("maxRelZ", maxRelZ);
    nhPrivate.getParam("disRatioZ", disRatioZ);

    tf_listener_ = new tf::TransformListener;

    ros::Subscriber subOdometry =
            nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
            "/registered_scan", 5, laserCloudHandler);

    ros::Subscriber subJoystick =
            nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

    ros::Subscriber subClearing =
            nh.subscribe<std_msgs::Float32>("/map_clearing", 5, clearingHandler);

    ros::Publisher pubLaserCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/terrain_map", 2);

    for (int i = 0; i < terrainVoxelNum; i++) {
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();

        if (newlaserCloud) {
            newlaserCloud = false;

            // terrain voxel roll over 
            float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

            while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) { 
                for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                            terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                              indY];
                    for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) { 
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
                    }
                    terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[indY]->clear();
                }
                terrainVoxelShiftX--;
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
                for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                            terrainVoxelCloud[indY];
                    for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
                    }
                    terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                      indY] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
                            ->clear();
                }
                terrainVoxelShiftX++;
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) { 
                for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                            terrainVoxelCloud[terrainVoxelWidth * indX +
                                              (terrainVoxelWidth - 1)];
                    for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
                    }
                    terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
                }
                terrainVoxelShiftY--;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            while (vehicleY - terrainVoxelCenY > terrainVoxelSize) { 
                for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                            terrainVoxelCloud[terrainVoxelWidth * indX];
                    for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
                        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
                    }
                    terrainVoxelCloud[terrainVoxelWidth * indX +
                                      (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
                    terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
                            ->clear();
                }
                terrainVoxelShiftY++;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            // stack registered laser scans 
            pcl::PointXYZI point;
            int laserCloudCropSize = laserCloudCrop->points.size();
            for (int i = 0; i < laserCloudCropSize; i++) {
                point = laserCloudCrop->points[i];

                int indX = int((point.x - vehicleX + terrainVoxelSize / 2) /
                               terrainVoxelSize) +
                           terrainVoxelHalfWidth; 
                int indY = int((point.y - vehicleY + terrainVoxelSize / 2) /
                               terrainVoxelSize) +
                           terrainVoxelHalfWidth;

                if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
                    indX--;
                if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
                    indY--; 

                if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
                    indY < terrainVoxelWidth) { 
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
                    terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++; 
                }
            }

            for (int ind = 0; ind < terrainVoxelNum; ind++) {
                if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre || 
                    laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                    voxelTimeUpdateThre || 
                    clearingCloud) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                            terrainVoxelCloud[ind];

                    laserCloudDwz->clear();
                    downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
                    downSizeFilter.filter(*laserCloudDwz); 

                    terrainVoxelCloudPtr->clear();
                    int laserCloudDwzSize = laserCloudDwz->points.size();
                    for (int i = 0; i < laserCloudDwzSize; i++) {
                        point = laserCloudDwz->points[i];
                        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                                         (point.y - vehicleY) * (point.y - vehicleY));
                        if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                            point.z - vehicleZ < maxRelZ + disRatioZ * dis && 
                            (laserCloudTime - systemInitTime - point.intensity <
                             decayTime || 
                             dis < noDecayDis) && 
                            !(dis < clearingDis && clearingCloud)) {
                            terrainVoxelCloudPtr->push_back(point);
                        }
                    }

                    terrainVoxelUpdateNum[ind] = 0;  
                    terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;  
                }
            }

            terrainCloud->clear();
            for (int indX = terrainVoxelHalfWidth - 5;
                 indX <= terrainVoxelHalfWidth + 5; indX++) {
                for (int indY = terrainVoxelHalfWidth - 5;
                     indY <= terrainVoxelHalfWidth + 5; indY++) {
                    *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
                }
            } 

             
            for (int i = 0; i < planarVoxelNum; i++) {
                planarVoxelElev[i] = 0;
                planarVoxelEdge[i] = 0;
                planarVoxelDyObs[i] = 0;
                planarPointElev[i].clear();
            }

            int terrainCloudSize = terrainCloud->points.size(); 
            for (int i = 0; i < terrainCloudSize; i++) {
                point = terrainCloud->points[i];

                int indX =
                        int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                        planarVoxelHalfWidth;
                int indY =
                        int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                        planarVoxelHalfWidth;

                if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                    indX--;
                if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                    indY--;
                 

                if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) { 
                    for (int dX = -1; dX <= 1; dX++) {
                        for (int dY = -1; dY <= 1; dY++) {
                            if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                                indY + dY >= 0 && indY + dY < planarVoxelWidth) {  
                                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                                        .push_back(point.z); 
                            }
                        }
                    }
                }

                if (clearDyObs) {
                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                        indY < planarVoxelWidth) {
                        float pointX1 = point.x - vehicleX;
                        float pointY1 = point.y - vehicleY;
                        float pointZ1 = point.z - vehicleZ;

                        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
                        if (dis1 > minDyObsDis) {
                            float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
                            if (angle1 > minDyObsAngle) {  
                                float pointX2 =
                                        pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                                float pointY2 =
                                        -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                                float pointZ2 = pointZ1;

                                float pointX3 =
                                        pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                                float pointY3 = pointY2;
                                float pointZ3 =
                                        pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                                float pointX4 = pointX3;
                                float pointY4 =
                                        pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                                float pointZ4 =
                                        -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                                float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;
                                if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV) { 
                                    planarVoxelDyObs[planarVoxelWidth * indX + indY]++;  
                                }
                            }
                        } else {
                            planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
                                    minDyObsPointNum; 
                        }
                    }
                }
            }

            if (clearDyObs) {
                for (int i = 0; i < laserCloudCropSize; i++) {
                    point = laserCloudCrop->points[i];

                    int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                                   planarVoxelSize) +
                               planarVoxelHalfWidth;
                    int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                                   planarVoxelSize) +
                               planarVoxelHalfWidth;

                    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                        indX--;
                    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                        indY--;

                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                        indY < planarVoxelWidth) {
                        float pointX1 = point.x - vehicleX;
                        float pointY1 = point.y - vehicleY;
                        float pointZ1 = point.z - vehicleZ;

                        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
                        float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
                        if (angle1 > minDyObsAngle) {
                            planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;  
                        }
                    }
                }
            }

            if (useSorting) {
                for (int i = 0; i < planarVoxelNum; i++) {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize > 0) {
                        sort(planarPointElev[i].begin(), planarPointElev[i].end());

                        int quantileID = int(quantileZ * planarPointElevSize);
                        if (quantileID < 0)
                            quantileID = 0;
                        else if (quantileID >= planarPointElevSize)
                            quantileID = planarPointElevSize - 1;

                        if (planarPointElev[i][quantileID] >
                            planarPointElev[i][0] + maxGroundLift &&
                            limitGroundLift) {
                            planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
                        } else {
                            planarVoxelElev[i] = planarPointElev[i][quantileID];
                        }
                    }
                }
            } else {
                for (int i = 0; i < planarVoxelNum; i++) {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize > 0) {
                        float minZ = 1000.0;
                        int minID = -1;
                        for (int j = 0; j < planarPointElevSize; j++) {
                            if (planarPointElev[i][j] < minZ) {
                                minZ = planarPointElev[i][j];
                                minID = j;
                            }
                        }

                        if (minID != -1) {
                            planarVoxelElev[i] = planarPointElev[i][minID]; 
                        }
                    }
                }
            }

            terrainCloudElev->clear();
            int terrainCloudElevSize = 0;
            for (int i = 0; i < terrainCloudSize; i++) {
                point = terrainCloud->points[i]; 
                if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) { 
                    int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                                   planarVoxelSize) +
                               planarVoxelHalfWidth;
                    int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                                   planarVoxelSize) +
                               planarVoxelHalfWidth;

                    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                        indX--;
                    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                        indY--;

                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                        indY < planarVoxelWidth) { 
                        if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                            minDyObsPointNum ||
                            !clearDyObs) {
                            float disZ =
                                    point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
                            if (considerDrop)
                                disZ = fabs(disZ);
                            int planarPointElevSize =
                                    planarPointElev[planarVoxelWidth * indX + indY].size();
                            if (disZ >= 0 && disZ < vehicleHeight &&
                                planarPointElevSize >= minBlockPointNum) {
                                terrainCloudElev->push_back(point);
                                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

                                terrainCloudElevSize++;
                            }
                        }
                    }
                }
            }

            if (noDataObstacle && noDataInited == 2) {  
                for (int i = 0; i < planarVoxelNum; i++) {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize < minBlockPointNum) {
                        planarVoxelEdge[i] = 1;
                    }
                }

                for (int noDataBlockSkipCount = 0;
                     noDataBlockSkipCount < noDataBlockSkipNum;
                     noDataBlockSkipCount++) {
                    for (int i = 0; i < planarVoxelNum; i++) {
                        if (planarVoxelEdge[i] >= 1) {
                            int indX = int(i / planarVoxelWidth);
                            int indY = i % planarVoxelWidth;
                            bool edgeVoxel = false;
                            for (int dX = -1; dX <= 1; dX++) {
                                for (int dY = -1; dY <= 1; dY++) {
                                    if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                                        indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                                        if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                                            dY] < planarVoxelEdge[i]) {
                                            edgeVoxel = true;
                                        }
                                    }
                                }
                            }

                            if (!edgeVoxel)
                                planarVoxelEdge[i]++;
                        }
                    }
                }

                for (int i = 0; i < planarVoxelNum; i++) {
                    if (planarVoxelEdge[i] > noDataBlockSkipNum) {
                        int indX = int(i / planarVoxelWidth);
                        int indY = i % planarVoxelWidth;

                        point.x =
                                planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
                        point.y =
                                planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
                        point.z = 0.0;
                        point.intensity = vehicleHeight;

                        point.x -= planarVoxelSize / 4.0;
                        point.y -= planarVoxelSize / 4.0;
                        terrainCloudElev->push_back(point);

                        point.x += planarVoxelSize / 2.0;
                        terrainCloudElev->push_back(point);

                        point.y += planarVoxelSize / 2.0;
                        terrainCloudElev->push_back(point);

                        point.x -= planarVoxelSize / 2.0;
                        terrainCloudElev->push_back(point);
                    }
                }
            }

            clearingCloud = false;

            // publish points with elevation
            sensor_msgs::PointCloud2 terrainCloud2;
            pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
            terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
            terrainCloud2.header.frame_id = "map";
            pubLaserCloud.publish(terrainCloud2);
        }

        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
