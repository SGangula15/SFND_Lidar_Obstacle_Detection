/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
    while(maxIterations--)
    {
        // Randomly sample subset and fit line

        std::unordered_set<int> inliers;
        while(inliers.size() <2 )
            inliers.insert(rand()%(cloud->points.size())); // randomly selecting between 0 and size of cloud

        float x1, y1, x2, y2;        //2D points
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;

        /* Equation of a line through two point in 2D is
         *  Ax+By+c = 0 (Line equation)
         *  Given two points: point1 (x1, y1) and point2 (x2, y2),
         *  the line through point1 and point2 has the specific form:
         *  (y1 -y2)x + (x2 -x1)y + (x1*y2 -x2*y1) = 0
         * */
        float a = (y1-y2);
        float b = (x2-x1);
        float c = (x1*y2 -x2*y1);

        // Measure distance between every point and fitted line
        for(int index = 0; index < cloud->points.size(); index++)
        {
            //Check every point is unique, if repeated skip
            if(inliers.count(index)>0)
                continue;

            pcl::PointXYZ point = cloud->points[index];
            float x3 = point.x;
            float y3 = point.y;

            /* equation for calculating distance between a point and line is
             * Distance d = abs(line equation)/sqrt(a^2+b^2)
             * */
            float absLineEquation = fabs(a*x3+b*y3+c); //absolute of line equation
            float d = absLineEquation/sqrt(a*a+b*b);

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

	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
    while(maxIterations--)
    {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;            
        for (int index = 0; index < 3; index++) // 3, because of 3D plane 
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
		float i = (((y2-y1)*(z3-z1)) -((z2-z1)*(y3-y1)));        
		float j = (((z2-z1)*(x3-x1)) -((x2-x1)*(z3-z1)));
		float k = (((x2-x1)*(y3-y1)) -((y2-y1)*(x3-x1)));
        float euclideanDist = sqrtf(i*i+j*j+k*k);

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
            float d = abs3DLineEquation/euclideanDist;

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

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
