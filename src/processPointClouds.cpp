// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // 1:Do a resolution reduction using Voxel Grid filter 
    typename pcl::PointCloud<PointT>::Ptr cloudLowResolution (new pcl::PointCloud<PointT>()); 
    pcl::VoxelGrid<PointT> reduceResolution;
    reduceResolution.setInputCloud(cloud);
    reduceResolution.setLeafSize(filterRes, filterRes, filterRes);  //filter resolution is set in meters
    reduceResolution.filter(*cloudLowResolution);

    // 2: Perform cropBox filter for region based croping
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>()); 
    pcl::CropBox<PointT> cropRegion(true);
    cropRegion.setMin(minPoint);
    cropRegion.setMax(maxPoint);
    cropRegion.setInputCloud(cloudLowResolution);
    cropRegion.filter(*cloudRegion);

    //3:optional: Remove the roof of the car using cropBox and get the indices
    //3a: get the roof of the car indices from cloudRegion
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    //3b: create a point cloud with rood indices so you can use extract function on it
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for(int point:indices){
        inliers->indices.push_back(point);
    }

    //3c: Now extract roof from the cloudRegion
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud( new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr planeCloud( new pcl::PointCloud<PointT> ());

  for(int index: inliers->indices)
  {
      planeCloud->points.push_back(cloud->points[index]);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
  //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    //Segment the largest planar component from the input point cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Cloud not estimate a planar model for the given point cloud dataset" << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}
// My Implementaion :
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// For max iterations 
    while(maxIterations--)
    {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;
        for (int index = 0; index < 3; index++) // 3, because of 3D plane 
        { 
            inliers.insert(rand() % cloud->points.size()); // randomly selecting between 0 and size of cloud
        }

        auto itr = inliers.begin();
        const float x1 = cloud->points[*itr].x;
        const float y1 = cloud->points[*itr].y;
		const float z1 = cloud->points[*itr].z;
        itr++;
        const float x2 = cloud->points[*itr].x;
        const float y2 = cloud->points[*itr].y;
		const float z2 = cloud->points[*itr].z;
		itr++;
		const float x3 = cloud->points[*itr].x;
		const float y3 = cloud->points[*itr].y;
		const float z3 = cloud->points[*itr].z;

         // Equation of a line through two point in 3D is
         //  Ax+By+Cz+D = 0 (3D Line equation)
         //  Given two points: point1 (x1, y1, z1), point2 (x2, y2, z2), and  point3 (x3, y3, z3)
         //  Use point1 as a reference and define two vectors on the plane v1 and v2: take cross product as shown in below link
         //  https://www.analyzemath.com/stepbystep_mathworksheets/vectors/cross_product.html
		 //  cross product = <i, j, k>
		 //  Coefficents of 3D Line equation will be: 
		 //  A = i, B = j, C = k, D = -(i*x1+j*y1+k*z1)
         
		const float i = static_cast<float>(((y2-y1)*(z3-z1)) -((z2-z1)*(y3-y1)));        
		const float j = static_cast<float>(((z2-z1)*(x3-x1)) -((x2-x1)*(z3-z1)));
		const float k = static_cast<float>(((x2-x1)*(y3-y1)) -((y2-y1)*(x3-x1)));

        const float A{i}, B{j}, C{k};
        const float D{ static_cast<float>(-((i * x1) + (j * y1) + (k * z1))) };

        // Euclidean Distance is sqrt(A^2+B^2+C^2)
        const float euclideanDistance{ sqrtf((A * A) + (B * B) + (C * C)) };

        // Measure distance between every point and fitted line
        for(int index = 0; index < cloud->points.size(); index++)
        {
            //Check every point is unique, if repeated skip
            if(inliers.count(index)>0)
                continue;

            //absolute of 3D line equation
            const float planeEquation{ std::fabs((A * cloud->points.at(index).x) + (B * cloud->points.at(index).y) +
                                         (C * cloud->points.at(index).z) + D) }; 
            
            // equation for calculating distance between a point and line is
            // Distance d = abs(3D line equation)/euclideanDistance
            float d = planeEquation/euclideanDistance;

            // If distance is smaller than threshold count it as inlier
            if(d <= distanceTol)
                inliers.insert(index);
        }

        // Return indicies of inliers from fitted line with most inliers
        if(inliers.size()>inliersResult.size())
        {
            inliersResult = inliers;
        }

    }

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersData,cloud);

    // Using ransac2d.cpp main() logic for coversion of std::unordered_set<int> inliers to 
    //  two point cloud objects
    //
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());   //Cloud Inliers
	typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());  //Cloud Outliers

	if(!inliersResult.empty())
    {
        for(int index = 0; index < cloud->points.size(); index++)
	    {
		PointT point = cloud->points.at(index);
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstacleCloud->points.push_back(point);
	    }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(
         obstacleCloud, planeCloud);
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Create the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud); //takes obstacle cloud

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); //
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices); //give indices of all clusters in cloud(object cloud would be passed as argument)

    //Now iterate through for number of clusters found in input cloud
    for(pcl::PointIndices getIndices: clusterIndices)
    {
        //create a cloudCluster to store all the points pointed from each clusterIndices
        //example:clusterIndices[0] is a vector which stores all the points belong to [0] extracted cluster 
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        //getIndices will have indices vectors for each cluster cloudCluster[0], cloudCluster[1]
        // and each indices vector store the complete points for the specific extracted cluster
        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense =  true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    //std::cout << "start time is " << startTime.count() <<endl;
    //std::cout << "end time is " << endTime.count() <<endl;
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::chrono::duration<long, std::micro> elapsedTimeMicro = elapsedTime;
    std::cout << "clustering took " << elapsedTimeMicro.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: adapt euclidean clustering function developed in quiz to 3d point cloud detected obstacles
    // use in built clustering method for reference

    // Pseudo Code
    // Create kdtree
    // Add point to kdtree
    // search for clusters with distance tolerance in kdtree
    // save cluster if it is within boundaries (min ,max)

    //Create the KdTree object for the search method of the extraction
    KdTree* tree = new KdTree;
    
    //insert points into kdtree
    std::vector<std::vector<float>> points;
    for (int i=0; i<cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        
        //create a point vector 
        //extract x,y,z from input point and store in point vector
        std::vector<float> pointVector;
        pointVector.push_back(point.x);
        pointVector.push_back(point.y);
        pointVector.push_back(point.z);
        
        //Now insert each point(x,y,z) into the kd tree
        tree->insert(pointVector, i); 
        points.push_back(pointVector);
    }

    // cluster points within the distance tolerane 
    std::vector<std::vector<int>> clustersIndices = euclideanCluster(points, tree, clusterTolerance);

    //ClusterIndices contains vector of clusters - cluster[0], [1]...
    //each cluster[x] is a pointer to the point cloud with list of all indices of the cluster
    for(std::vector<int> eachCluster : clustersIndices)
    {
        //extract point cloud of each cluster in clusterCloud
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for (int indice: eachCluster)
        {
            clusterCloud->points.push_back(cloud->points[indice]);
        }
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        //check for boundaries of clusters 
        if ((clusterCloud->width >= minSize) and (clusterCloud->width <= maxSize))
            clusters.push_back(clusterCloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}