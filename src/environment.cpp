/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar lidar = Lidar(cars, 0.0);
    auto pointCloud = lidar.scan();

    ProcessPointClouds<pcl::PointXYZ> processor;
    auto segmented = processor.SegmentPlane(pointCloud, 100, 0.2);
    // renderPointCloud(viewer, segmented.first, "planeCloud", Color(0,1,0));
    // renderPointCloud(viewer, segmented.second, "obstCloud", Color(1,0,0));

    auto clusters = processor.Clustering(segmented.second, 2.0, 3, 30);
    int clusterId = 0;
    // std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0)};
    std::vector<Color> colors = {Color(1,1,0)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
    {
        if (clusterId >= 4)
            break;

        std::cout << "cluster size ";
        processor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = processor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    //renderBox(viewer,box,clusterId);
    // renderRays(viewer, lidar.position, pointCloud);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud =
        pointProcessorI->FilterCloud(inputCloud, 0.2f, Eigen::Vector4f (-40.0, -5.0, -2.0, 1), Eigen::Vector4f (40.0, 6.0, 10.0, 1));

    auto segmented = pointProcessorI->SegmentPlane(filterCloud, 50, 0.3);
    renderPointCloud(viewer, segmented.first, "planeCloud", Color(0,1,0));
    // renderPointCloud(viewer, segmented.second, "obstCloud", Color(1,0,0));
    // renderPointCloud(viewer,filterCloud,"filterCloud");

    auto clusters = pointProcessorI->Clustering(segmented.second, 0.5, 5, 1000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,1,0)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%(colors.size())]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}