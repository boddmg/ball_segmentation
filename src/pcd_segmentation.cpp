#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include "OpenNI_Motor.h"
#include <string>
#include <iostream>
#include <vector>
#include <windows.h>
#include <conio.h>
#include <ctime> 


using namespace std;

typedef pcl::PointXYZRGB PointT;

//机械臂显示类
class RobotViewer
{
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;//显示窗口的智能指针
	RobotViewer()
	{
	// ------------------------------------
	// -----Create blank point cloud-----
	// ------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<PointT>::Ptr point_cloud_ptr (new pcl::PointCloud<PointT>);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerTemp(new pcl::visualization::PCLVisualizer ("Robot Viewer"));

	viewerTemp->setBackgroundColor (0, 0, 0);

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(point_cloud_ptr);						//创建一个空白的点云,并取得

	viewerTemp->addPointCloud<PointT> (point_cloud_ptr, rgb, "kinect");
	viewerTemp->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "kinect");


	Eigen::Affine3f transform;									//变换用仿射矩阵
	std::vector< pcl::visualization::Camera > cameras;

	viewerTemp->addCoordinateSystem(100.0);
	viewerTemp->initCameraParameters();
	viewerTemp->setCameraPosition(0,0,-2500,0,1,0);				//相机位姿变换
	viewer=viewerTemp;
	}

	void spinOnce()
	{
		viewer->spinOnce();
	}
	void updateBGcloud(pcl::PointCloud<PointT> &_cloud)
	{
		viewer->updatePointCloud(_cloud.makeShared(),"kinect");
	}

};


//点云获取类
class pointCloudGrabber
{
protected:
	bool isUpdatedFlag;
	pcl::PointCloud<PointT> cloud;
	void tansform(pcl::PointCloud<PointT> &_cloud)
	{
		Eigen::Matrix4f transformMatrix;
		transformMatrix<<
					1000,0,0,0,
					0,1000,0,0,
					0,0,1000,0,
					0,0,0,1
					;
		transformMatrix=pcl::getTransformation(0,0,0,0,0,M_PI).matrix()*transformMatrix;
		pcl::transformPointCloud(_cloud,_cloud,transformMatrix);
	}
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
	pointCloudGrabber()
	{
		isUpdatedFlag=false;
	}
//	virtual void grab()=0;

	pcl::PointCloud<PointT> getPointCloud()
	{
		return cloud;
	}


	bool isUpdated()
	{
		if(isUpdatedFlag)
		{
			isUpdatedFlag=false;
			return true;
		}
		return false;
	}
};

//从文件获取点云，继承自pointCloudGrabber
class FileGrabber:public pointCloudGrabber
{
private:
	string fileName;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
	FileGrabber(string _fileName):fileName(_fileName)
	{
	}
	FileGrabber()
	{
	}

	void setFileName(string _fileName)
	{
		fileName=_fileName;
	}

	void grab()
	{
		pcl::io::loadPCDFile<PointT>(fileName,cloud);
		isUpdatedFlag=true;
	}
};

string getNowTimeStr()
{
	stringstream timeS;
	timeS<<(uint64_t)time(NULL);
	string nowTime(timeS.str());
	return nowTime;
}

#define CLAMP_COLOR_VALUE(v) (v) = (v)/255;

#define MAX(x,y) (x)>(y)?(x):(y)
#define MIN(x,y) (x)<(y)?(x):(y)

#define fequ(x,y,tol) ( ((x)*(1-(tol))<(y))&&((x)*(1+(tol))>(y)) )

void RGB2HSL(float r, float g, float b,
             float *h, float *s, float *l) {
    CLAMP_COLOR_VALUE(r);
    CLAMP_COLOR_VALUE(g);
    CLAMP_COLOR_VALUE(b);
    
    float max, min, delta, sum;
    max = MAX(r, MAX(g, b));
    min = MIN(r, MIN(g, b));
    delta = max - min;
    sum = max + min;
    
    *l = sum / 2;           // Lightness
    if (delta == 0) {       // No Saturation, so Hue is undefined (achromatic)
        *h = *s = 0;
        return;
    }
    *s = delta / (sum < 1 ? sum : 2 - sum);             // Saturation
    if      (r == max) *h = (g - b) / delta / 6;        // color between y & m
    else if (g == max) *h = (2 + (b - r) / delta) / 6;  // color between c & y
    else               *h = (4 + (r - g) / delta) / 6;  // color between m & y
    if (*h < 0) *h += 1;
}

int main (int argc, char** argv)
{
	uint32_t i=0;
	KinectMotors m_kinectMotors;		//OpenNI_Motor.cpp里的用来控制kinect转角的类
	m_kinectMotors.Open();
	m_kinectMotors.Move(0);				//控制转角，安全范围：-60~60度，0度为水平
	FileGrabber fileGrabber;
	RobotViewer viewer1;
	RobotViewer viewer2;

	cout<<argc<<endl;
	if(argc>=2)
	{
		fileGrabber.setFileName(argv[1]);
		fileGrabber.grab();
		 
		cout<<getNowTimeStr()<<"Set fliter params"<<endl;
		pcl::PointCloud<PointT> _cloud(fileGrabber.getPointCloud());
		pcl::PointCloud<PointT> filteredCloud;

		for (size_t i=0;i<_cloud.points.size();++i)
		{
			float h,s,l;
			float tol=0.1;
			RGB2HSL(_cloud.points[i].r,_cloud.points[i].g,_cloud.points[i].b,&h,&s,&l);
			if(!fequ(h,60.0/360,0.1) )
			{
				_cloud.points[i].r=0;
			}
		}
		cout<<getNowTimeStr()<<"color filter"<<endl;
		cout<<_cloud.points.size()<<endl;
		//过滤球的颜色以外的对象
		pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());

		range_cond->addComparison (pcl::PackedRGBComparison<PointT>::ConstPtr (new
		  pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::GT, 1)));

		// build the filter
		pcl::ConditionalRemoval<PointT> condrem (range_cond);
		condrem.setInputCloud(_cloud.makeShared());
		condrem.setKeepOrganized(false);
		// apply filter
		condrem.filter(filteredCloud);
		cout<<getNowTimeStr()<<"over"<<endl;

		cout<<filteredCloud.points.size()<<endl;
		
		cout<<getNowTimeStr()<<"RadiusOutlierRemoval"<<endl;
		//过滤离群的点
		pcl::RadiusOutlierRemoval<PointT> outrem(true);
		// build the filter
		outrem.setInputCloud(filteredCloud.makeShared());
		outrem.setRadiusSearch(10);
		outrem.setMinNeighborsInRadius (5);
		outrem.setKeepOrganized(false);
		// apply filter
		outrem.filter (filteredCloud);

		cout<<getNowTimeStr()<<"over"<<endl;
		
		
		//Start to segmentation
		pcl::SACSegmentation<PointT> seg; 
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
		pcl::ExtractIndices<PointT> extract;
		pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
		pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());

		cout<<getNowTimeStr()<<"Set segment params"<<endl;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_SPHERE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (5);
		seg.setRadiusLimits (0, 100);
		seg.setInputCloud (filteredCloud.makeShared());
		cout<<getNowTimeStr()<<"Set segment params done...."<<endl;

		cout<<getNowTimeStr()<<"Start segment"<<endl;
		seg.segment (*inliers_sphere, *coefficients_sphere);
		cout<<getNowTimeStr()<<"Segment done...."<<endl;

		cout<<getNowTimeStr()<<"Start extract"<<endl;

		extract.setInputCloud (filteredCloud.makeShared());
		extract.setIndices (inliers_sphere);
		extract.setNegative (false);
		extract.filter (filteredCloud);
		cout<<getNowTimeStr()<<"Extract done....."<<endl;
		

		viewer1.updateBGcloud(_cloud);
		viewer2.updateBGcloud(filteredCloud);
	}

	while(true)
	{
		viewer1.spinOnce();
		viewer2.spinOnce();
		if(kbhit())
		{
			switch (getch())
			{
			case 32:
				{
					//空格键
				}break;

			case 27:
				{
					return 0;
					//ESC键
				}break;

			case 13:
				{
					//回车键
				}break;

			}
		}
	}
	return 0;
}
