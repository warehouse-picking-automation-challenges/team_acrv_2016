#ifndef KINFU_TOOLS_H
#define KINFU_TOOLS_H

#include "ros_kinfu.h"

namespace kinfu_tools {

void writePolygonMeshFile(int format, const pcl::PolygonMesh& mesh);

int print_cli_help();

void writePolygonMeshFile(int format, const pcl::PolygonMesh& mesh);

boost::shared_ptr<pcl::PolygonMesh> convertToMesh(
    const pcl::gpu::DeviceArray<pcl::PointXYZ>& triangles);

void clearClouds(bool print_message = false);

void toggleNormals();

/*template<typename CloudT> void
writeCloudFile (int format, const CloudT& cloud);


template<typename CloudPtr> void writeCloudFile (int format, const CloudPtr&
cloud_prt);

template<typename MergedT, typename PointT>
typename PointCloud<MergedT>::Ptr merge(const PointCloud<PointT>& points, const
PointCloud<RGB>& colors)
{
  typename PointCloud<MergedT>::Ptr merged_ptr(new PointCloud<MergedT>());

  pcl::copyPointCloud (points, *merged_ptr);
  for (size_t i = 0; i < colors.size (); ++i)
    merged_ptr->points[i].rgba = colors.points[i].rgba;

  return merged_ptr;
}*/
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif  // KINFU_TOOLS_H
