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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

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
        while(inliers.size() < 3 ) // 3, because of 3D plane 
            inliers.insert(rand()%(cloud->points.size())); // randomly selecting between 0 and size of cloud

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;  //3D points
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

        /* Equation of a line through two point in 3D is
         *  Ax+By+Cz+D = 0 (3D Line equation)
         *  Given two points: point1 (x1, y1, z1), point2 (x2, y2, z2), and  point3 (x3, y3, z3)
         *  Use point1 as a reference and define two vectors on the plane v1 and v2: take cross product as shown in below link
         *  https://www.analyzemath.com/stepbystep_mathworksheets/vectors/cross_product.html
		 *  cross product = <i, j, k>
		 *  Coefficents of 3D Line equation will be: 
		 *  A = i, B = j, C = k, D = -(i*x1+j*y1+k*z1)
         * */
		float i = (((y2-y1)*(z3-z1)) -((z2-z1)-(y3-y1)));        
		float j = (((z2-z1)*(x3-x1)) -((x2-x1)-(z3-z1)));
		float k = (((x2-x1)*(y3-y1)) -((y2-y1)-(x3-x1)));

        // Measure distance between every point and fitted line
        for(int index = 0; index < cloud->points.size(); index++)
        {
            //Check every point is unique, if repeated skip
            if(inliers.count(index)>0)
                continue;

            pcl::PointXYZ point = cloud->points[index];
            float x3 = point.x;
            float y3 = point.y;
			float z3 = point.z;

            /* equation for calculating distance between a point and line is
             * Distance d = abs(3D line equation)/sqrt(A^2+B^2+C^2)
             * */
            float abs3DLineEquation = fabs(i*x3+j*y3+k*z3-(i*x1+j*y1+k*z1)); //absolute of 3D line equation
            float d = abs3DLineEquation/sqrt(i*i+j*j+k*k);

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

    /* Using ransac2d.cpp main() logic for coversion of std::unordered_set<int> inliers to 
    *  two point cloud objects
    */
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());   //Cloud Inliers
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());  //Cloud Outliers

	while(!inliersResult.empty())
    {
        for(int index = 0; index < cloud->points.size(); index++)
	    {
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			obstCloud->points.push_back(point);
		else
			planeCloud->points.push_back(point);
	    }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(
        obstCloud, planeCloud);

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

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