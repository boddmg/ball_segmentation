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
#include <iomanip>

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


#define MAX(x,y) (x)>(y)?(x):(y)
#define MIN(x,y) (x)<(y)?(x):(y)


class BallRecongniter
{
public:
void CLAMP_COLOR_VALUE(float &v)
	{
		v=v/255.0;
	}

	inline bool fequ(float x,float y,float tol) 
	{
		return 	( ((x)*(1-(tol))<(y))&&((x)*(1+(tol))>(y)) );
	}

	inline void RGB2HSL(float r, float g, float b,
		float *h, float *s, float *l) 
	{
		CLAMP_COLOR_VALUE(r);
		CLAMP_COLOR_VALUE(g);
		CLAMP_COLOR_VALUE(b);

		float max, min, delta, sum;
		max = MAX(r, MAX(g, b));
		min = MIN(r, MIN(g, b));
		delta = max - min;
		sum = max + min;

		*l = sum / 2;           // Lightness
		if (delta == 0) 
		{       // No Saturation, so Hue is undefined (achromatic)
			*h = *s = 0;
			return;
		}
		*s = delta / (sum < 1 ? sum : 2 - sum);             // Saturation
		if      (r == max) *h = (g - b) / delta / 6;        // color between y & m
		else if (g == max) *h = (2 + (b - r) / delta) / 6;  // color between c & y
		else               *h = (4 + (r - g) / delta) / 6;  // color between m & y
		if (*h < 0) *h += 1;
	}

	void FilterWithColorInHSL(pcl::PointCloud<PointT> &inCloud,pcl::PointCloud<PointT> &outCloud,float inH,float inTol)
	{
		//过滤球的颜色以外的对象
		for (pcl::PointCloud<PointT>::iterator i=inCloud.begin();i!=inCloud.end();++i)
		{
			float h,s,l;
			float tol=0.1;
			RGB2HSL(i->r,i->g,i->b,&h,&s,&l);

			if(fequ(h,inH,inTol) && pcl::isFinite(*i))
			{
				//i->r=0;
				outCloud.insert(outCloud.end(),*i);
			}
		}	
	}

	void FilterRemovalPointsWithRadius(pcl::PointCloud<PointT> &inCloud,pcl::PointCloud<PointT> &outCloud,float radiusSearch,float minNeighborsInRadius)
	{
		pcl::RadiusOutlierRemoval<PointT> outrem(true);
		// build the filter
		outrem.setInputCloud(inCloud.makeShared());
		outrem.setRadiusSearch(radiusSearch);
		outrem.setMinNeighborsInRadius (minNeighborsInRadius);
		outrem.setKeepOrganized(false);
		// apply filter
		outrem.filter (outCloud);
	}

	void SphereSegmentation(pcl::PointCloud<PointT> &inCloud,pcl::PointCloud<PointT> &outCloud,pcl::ModelCoefficients &coefficients_sphere)
	{
		pcl::SACSegmentation<PointT> seg; 
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
		pcl::ExtractIndices<PointT> extract;
		pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
		pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_SPHERE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (5);
		seg.setRadiusLimits (20, 40);
		seg.setInputCloud (inCloud.makeShared());
		seg.segment (*inliers_sphere, coefficients_sphere);
		
		extract.setInputCloud (inCloud.makeShared());
		extract.setIndices (inliers_sphere);
		extract.setNegative (false);
		extract.filter (outCloud);
	}

	void getSphere(pcl::PointCloud<PointT> &inCloud,pcl::PointCloud<PointT> &outCloud,pcl::ModelCoefficients &coefficients_sphere)
	{
		FilterWithColorInHSL(inCloud,outCloud,0.1667,0.1);//Filter with color in HSL. To change the color, change the params of the function.
		FilterRemovalPointsWithRadius(outCloud,outCloud,5,5);
		SphereSegmentation(outCloud,outCloud,coefficients_sphere);
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

//从Kinect获取点云，继承自pointCloudGrabber
class KinectGrabber:public pointCloudGrabber
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

	RobotViewer robotViewer;
	string path;
	bool isSaving;

	KinectGrabber()
	{
		isSaving=false;
		pcl::Grabber* interface = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
		boost::bind (&KinectGrabber::kinect_callback, this, _1,interface);

		interface->registerCallback (f);

		interface->start();
	}

	void kinect_callback (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &_cloud,pcl::Grabber* interface)
	{
//		Sleep(100);
		cloud=*_cloud;
		tansform(cloud);
		if(isSaving==true)
		{
			isSaving=false;
			saveAsPCDFile(path);
		}

		
		pcl::PointCloud<PointT> sphere;
		pcl::ModelCoefficients coefficients_sphere;
		BallRecongniter ballRecongniter;
		ballRecongniter.getSphere(cloud,sphere,coefficients_sphere);
		robotViewer.viewer->updatePointCloud(sphere.makeShared(),"kinect");
	}

	void saveAsPCDFile(string _path)
	{
		pcl::PCDWriter writer;
		writer.writeBinaryCompressed(_path,cloud);
	}
	 
	void grab(string _path)
	{
		path=_path;
		isSaving=true;
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
	static DWORD lastTime=GetTickCount();
	DWORD nowTime= GetTickCount();
	stringstream stringBuilder;
	stringBuilder<< nowTime;
	stringBuilder<<"......";
	stringBuilder<<setw(4)<<(nowTime-lastTime)<<"  ";
	lastTime=nowTime;
	string timeStr(stringBuilder.str());
	return timeStr;
}

void debug_printf(string text)
{
	static bool firstOpen=true;
	ofstream oFile("log.txt",ios::app);
	string timeStr=getNowTimeStr();
	if (firstOpen)
	{
		char time[128],date[128];
		_strtime(time); 
		_strdate(date); 
		oFile<<"-------------------"<<date<<" "<<time<<"------------------"<<endl;//debug
		firstOpen=false;
	}
	oFile<<timeStr<<text<<endl;//debug
	cout<<timeStr<<text<<endl;//debug
}



int main (int argc, char** argv)
{
	uint32_t i=0;
	KinectMotors m_kinectMotors;		//OpenNI_Motor.cpp里的用来控制kinect转角的类
	m_kinectMotors.Open();
	m_kinectMotors.Move(0);				//控制转角，安全范围：-60~60度，0度为水平
	stringstream stringBuilder;
	KinectGrabber kinectGrabber;      
	while(true)
	{
		kinectGrabber.robotViewer.viewer->spinOnce();
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
