#include "kinfu_tools.h"

namespace kinfu_tools
{
    int print_cli_help ()
    {
      cout << "\nKinFu parameters:" << endl;
      cout << "    --help, -h                              : print this message" << endl;
      cout << "    --registration, -r                      : try to enable registration (source needs to support this)" << endl;
      cout << "    --current-cloud, -cc                    : show current frame cloud" << endl;
      cout << "    --save-views, -sv                       : accumulate scene view and save in the end ( Requires OpenCV. Will cause 'bad_alloc' after some time )" << endl;
      cout << "    --integrate-colors, -ic                 : enable color integration mode (allows to get cloud with colors)" << endl;
      cout << "    --scale-truncation, -st                 : scale the truncation distance and raycaster based on the volume size" << endl;
      cout << "    -volume_size <size_in_meters>           : define integration volume size" << endl;
      cout << "    --depth-intrinsics <fx>,<fy>[,<cx>,<cy> : set the intrinsics of the depth camera" << endl;
      cout << "    -save_pose <pose_file.csv>              : write tracked camera positions to the specified file" << endl;
      cout << "Valid depth data sources:" << endl;
      cout << "    -dev <device> (default), -oni <oni_file>, -pcd <pcd_file or directory>" << endl;
      cout << "";
      cout << " For RGBD benchmark (Requires OpenCV):" << endl;
      cout << "    -eval <eval_folder> [-match_file <associations_file_in_the_folder>]" << endl;

      return 0;
    }




    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void
    writePolygonMeshFile (int format, const pcl::PolygonMesh& mesh)
    {
      if (format == ros_kinfu::MESH_PLY)
      {
        cout << "Saving mesh to to 'mesh.ply'... " << flush;
        pcl::io::savePLYFile("mesh.ply", mesh);
      }
      else /* if (format == KinFuApp::MESH_VTK) */
      {
        cout << "Saving mesh to to 'mesh.vtk'... " << flush;
        pcl::io::saveVTKFile("mesh.vtk", mesh);
      }
      cout << "Done" << endl;
    }


    boost::shared_ptr<pcl::PolygonMesh> convertToMesh(const DeviceArray<PointXYZ>& triangles)
    {
      if (triangles.empty())
          return boost::shared_ptr<pcl::PolygonMesh>();

      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.width  = (int)triangles.size();
      cloud.height = 1;
      triangles.download(cloud.points);

      boost::shared_ptr<pcl::PolygonMesh> mesh_ptr( new pcl::PolygonMesh() );
      pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);

      mesh_ptr->polygons.resize (triangles.size() / 3);
      for (size_t i = 0; i < mesh_ptr->polygons.size (); ++i)
      {
        pcl::Vertices v;
        v.vertices.push_back(i*3+0);
        v.vertices.push_back(i*3+2);
        v.vertices.push_back(i*3+1);
        mesh_ptr->polygons[i] = v;
      }
      return mesh_ptr;
    }


}

///////////////////////////////////////////////////////////////////////////////


