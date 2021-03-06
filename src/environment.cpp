/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
//#include "quiz/cluster/cluster.h" //seems redundant as cpp is included
// to avoid linker error for euclideanCluster add .cpp
#include "quiz/cluster/cluster.cpp"    
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

//Compiler switch for single and stream pcd processing
//#define SINGLE_PCD
#define STREAM_PCD

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
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TO DO Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);
    //Scan to get the point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud = lidar->scan();
    //render the point cloud data to visualize
    //renderPointCloud(viewer, inputPointCloud, "pointCloud");
    // Create point processor
    //On Stack
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    //On heap
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.RansacPlane(inputPointCloud, 100, 0.3);
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputPointCloud, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    
    //Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        //render box
        //Box box = pointProcessor.BoundingBox(cluster);
        //renderBox(viewer, box, clusterId, colors[clusterId], 1);

        ++clusterId;
    }    
}

#ifdef SINGLE_PCD
//process single pcd 
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  // Experiment with the min and max Point x,y,z values and find what works best
  Eigen::Vector4f minPoint (-30, -5.6, -3, 1);
  Eigen::Vector4f maxPoint (30, 6.8, 10, 1);
  float filterResolution (0.15); //filter resolution is in meters
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterResolution , minPoint, maxPoint); 
  //renderPointCloud(viewer,filterCloud,"filterCloud");
  
  // Segment the filtered cloud using RANSAC Plane
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentFilteredCloud = pointProcessorI->RansacPlane(filterCloud, 100, 0.3);
  //renderPointCloud(viewer,segmentFilteredCloud.first,"obstCloud",Color(1,0,0));
  renderPointCloud(viewer,segmentFilteredCloud.second,"planeCloud",Color(0,1,0));

  //Apply Euclidean Clustering on segmented point cloud
  //Clustering
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentFilteredCloud.first, 0.6, 420, 4800);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentFilteredCloud.first, 0.7, 5, 4800);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        //render box
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId], 1);

        ++clusterId;
    } 
}
#endif //single pcd

#ifdef STREAM_PCD
//Process stream of pcd data
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  // Experiment with the min and max Point x,y,z values and find what works best
  Eigen::Vector4f minPoint (-20, -5, -3, 1);
  Eigen::Vector4f maxPoint (20, 6.5, 6, 1);
  float filterResolution (0.28); //filter resolution is in meters
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterResolution , minPoint, maxPoint); 
  //renderPointCloud(viewer,filterCloud,"filterCloud");
  
  //brief: Segment the filtered cloud using RANSAC Plane
  //details: [in] second parameter - max iterations
  //[in] third parameter “distance threshold”, determines how close a point must be to the model in order to be considered an inlier.
  //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentFilteredCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.3);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentFilteredCloud = pointProcessorI->RansacPlane(filterCloud, 150, 0.2);
  //renderPointCloud(viewer,segmentFilteredCloud.first,"obstCloud",Color(1,0,0));
  renderPointCloud(viewer,segmentFilteredCloud.second,"planeCloud",Color(0,1,0));

  //breif: Apply Euclidean Clustering on segmented point cloud
  //details: [in] 2nd parameter ClusterTolerance() - If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
  //On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster.
    //Using pcl clustering library
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentFilteredCloud.first, 0.6, 10, 5200);
    //Clustering function developed in Quiz 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentFilteredCloud.first, 0.42, 5, 800);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        //render box
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1,0,0), 1); 

        ++clusterId;
    } 
}
#endif //Stream pcd

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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    #ifdef SINGLE_PCD
    cout<<" Visualizing Single PCD data" << endl;
    cityBlock(viewer);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    #endif //single pcd

    #ifdef STREAM_PCD
    cout<<" Visualizing Stream of PCD data" << endl;
    //process stream of pcd data
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
    #endif //Stream PCD
    
    
}
