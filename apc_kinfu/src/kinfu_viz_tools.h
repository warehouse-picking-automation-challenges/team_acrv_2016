#ifndef KINFU_VIZ_TOOLS_H
#define KINFU_VIZ_TOOLS_H

#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/gpu/kinfu/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/exceptions.h>

#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/common/angles.h>
#include <pcl/gpu/kinfu/tools/camera_pose.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;

namespace kinfu_viz
{
    vector<string> getPcdFilesInDir(const string& directory);
    void setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);
    Eigen::Affine3f getViewerPose (visualization::PCLVisualizer& viewer);
    Eigen::Affine3f getViewerPose (visualization::PCLVisualizer& viewer);
    void setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);
}


typedef pcl::ScopeTime ScopeTimeT;

class SampledScopeTime : public StopWatch
{
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms);
  ~SampledScopeTime();
private:
  int& time_ms_;
};

/////////////////////////////////////////////////////////////////////////////////

class ImageView
{
    public:
    ImageView(int viz);
    void showScene (KinfuTracker& kinfu, cv::Mat &rgb, bool registration, Eigen::Affine3f* pose_ptr = 0);
    void showScene (KinfuTracker& kinfu, bool registration, Eigen::Affine3f* pose_ptr = 0);
    void showDepth (const sensor_msgs::Image::ConstPtr depth);
    void generateDepth (KinfuTracker& kinfu, const Eigen::Affine3f& pose, KinfuTracker::DepthMap &generated_depth_);
    void toggleImagePaint();

    int viz_;
    bool paint_image_;
    bool accumulate_views_;

    visualization::ImageViewer::Ptr viewerScene_;
    visualization::ImageViewer::Ptr viewerDepth_;
    //visualization::ImageViewer viewerColor_;

    KinfuTracker::View view_device_;
    KinfuTracker::View colors_device_;
    vector<KinfuTracker::PixelRGB> view_host_;

    RayCaster::Ptr raycaster_ptr_;

    vector<cv::Mat> views_;
};

////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace gpu
  {
    void paint3DView (const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);
    void mergePointNormal (const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output);
    void mergePointRGB (const DeviceArray<PointXYZ>& cloud, const DeviceArray<RGB>& rgb, DeviceArray<PointXYZRGB>& output);
  }

  namespace visualization
  {
    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGB handler class for colors. Uses the data present in the "rgb" or "rgba"
      * fields from an additional cloud as the color at each point.
      * \author Anatoly Baksheev
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerRGBCloud : public PointCloudColorHandler<PointT>
    {
      using PointCloudColorHandler<PointT>::capable_;
      using PointCloudColorHandler<PointT>::cloud_;

      typedef typename PointCloudColorHandler<PointT>::PointCloud::ConstPtr PointCloudConstPtr;
      typedef typename pcl::PointCloud<RGB>::ConstPtr RgbCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRGBCloud<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRGBCloud<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRGBCloud (const PointCloudConstPtr& cloud, const RgbCloudConstPtr& colors)
          : rgb_ (colors)
        {
          cloud_  = cloud;
          capable_ = true;
        }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const
        {
          if (!capable_ || !cloud_)
            return (false);

          if (!scalars)
            scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
          scalars->SetNumberOfComponents (3);

          vtkIdType nr_points = vtkIdType (cloud_->points.size ());
          reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
          unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->GetPointer (0);

          // Color every point
          if (nr_points != int (rgb_->points.size ()))
            std::fill (colors, colors + nr_points * 3, static_cast<unsigned char> (0xFF));
          else
            for (vtkIdType cp = 0; cp < nr_points; ++cp)
            {
              int idx = cp * 3;
              colors[idx + 0] = rgb_->points[cp].r;
              colors[idx + 1] = rgb_->points[cp].g;
              colors[idx + 2] = rgb_->points[cp].b;
            }
          return (true);
        }

      private:
        virtual std::string
        getFieldName () const { return ("additional rgb"); }
        virtual std::string
        getName () const { return ("PointCloudColorHandlerRGBCloud"); }

        RgbCloudConstPtr rgb_;
    };
  }
}


/////////////////////////////////////////////////////////////////////////////////

class CurrentFrameCloudView
{
public:
    CurrentFrameCloudView();
    void show (const KinfuTracker& kinfu);
    void setViewerPose (const Eigen::Affine3f& viewer_pose);
    PointCloud<PointXYZ>::Ptr cloud_ptr_;
    DeviceArray2D<PointXYZ> cloud_device_;
    visualization::PCLVisualizer cloud_viewer_;
};

////////////////////////////////////////////////////////////////////////////////

class SceneCloudView
{
public:
  enum { GPU_Connected6 = 0, CPU_Connected6 = 1, CPU_Connected26 = 2 };

  SceneCloudView(int viz);

  void show (KinfuTracker& kinfu, bool integrate_colors);

  void generateCloud(KinfuTracker& kinfu, bool integrate_colors);

  void generateXYZRGB(PointCloud<PointXYZ>::Ptr cloud_ptr, PointCloud<RGB>::Ptr rgb_ptr_, PointCloud<PointXYZRGB>::Ptr output);

  void toggleCube(const Eigen::Vector3f& size);

  void toggleExtractionMode ();

  void toggleNormals ();

  void clearClouds (bool print_message = false);

  void showMesh(KinfuTracker& kinfu, bool /*integrate_colors*/);

  int viz_;
  int extraction_mode_;
  bool compute_normals_;
  bool valid_combined_;
  bool cube_added_;

  Eigen::Affine3f viewer_pose_;

  visualization::PCLVisualizer::Ptr cloud_viewer_;

  PointCloud<PointXYZ>::Ptr cloud_ptr_;
  PointCloud<Normal>::Ptr normals_ptr_;

  DeviceArray<PointXYZ> cloud_buffer_device_;
  DeviceArray<Normal> normals_device_;

  PointCloud<PointNormal>::Ptr combined_ptr_;
  DeviceArray<PointNormal> combined_device_;

  DeviceArray<RGB> point_colors_device_;
  PointCloud<RGB>::Ptr point_colors_ptr_;
  DeviceArray<PointXYZRGB> combined_color_device_;
  PointCloud<PointXYZRGB>::Ptr combined_color_ptr_;

  MarchingCubes::Ptr marching_cubes_;
  DeviceArray<PointXYZ> triangles_buffer_device_;

  boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;

  visualization::PointCloudColorHandlerRGBCloud<PointXYZRGB>::Ptr cloud_rgb_ptr_;

};



#endif // KINFU_VIZ_TOOLS_H
