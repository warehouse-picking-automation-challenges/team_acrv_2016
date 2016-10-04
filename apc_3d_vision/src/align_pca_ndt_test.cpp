/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <apc_3d_vision.hpp>

#include <string>

int main(int argc, char** argv) {
    Apc3dVision apc_vis;

    // NDT Params
    double leaf = 0.005;
    double transformationEpsilon = 0.001;
    double stepSize = 0.04;
    double resolution = 0.1;
    int maxIterations = 100;

    // Smoothing Params
    // 3cm
    double smoothingRadius = 0.03;

    // Statistical outlier removal params
    // 10 nearest points
    double outlierNumberOfSamples = 10;
    // any points passed 2 std devs of query point are removed
    double stdDevMulThresh = 2;

    leaf = std::atof(argv[4]);
    transformationEpsilon = std::atof(argv[5]);
    stepSize = std::atof(argv[6]);
    resolution = std::atof(argv[7]);
    maxIterations = std::atof(argv[8]);
    smoothingRadius = std::atof(argv[9]);
    outlierNumberOfSamples = std::atof(argv[10]);
    stdDevMulThresh = std::atof(argv[11]);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> modelPC;
    modelPC.reset(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> rawPC;
    rawPC.reset(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>> rawPCNormal;
    rawPCNormal.reset(new pcl::PointCloud<pcl::PointNormal>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> registeredPC;
    registeredPC.reset(new pcl::PointCloud<pcl::PointXYZ>);

    std::string modelFileName = (std::string)argv[1];
    std::string rawPCFileName = (std::string)argv[2];
    std::string outputModelFileName = (std::string)argv[3];

    bool isSuccess = false;

    isSuccess = apc_vis.load_pcd_file(modelFileName, modelPC);
    isSuccess = apc_vis.load_pcd_file(rawPCFileName, rawPC);

    // Remove statistical outliers
    apc_vis.outlier_removal(
        rawPC, rawPC, outlierNumberOfSamples, stdDevMulThresh);

    // Smooth the raw point cloud
    apc_vis.smooth_cloud(rawPC, rawPCNormal, smoothingRadius);

    // Replace rawPC with rawPCNormal points
    pcl::copyPointCloud(*rawPCNormal, *rawPC);
    //
    // isSuccess = apc_vis.save_pcd_file("model_demeaned.pcd", modelPC);
    // isSuccess = apc_vis.save_pcd_file("raw_demeaned.pcd", rawPC);

    // input, target
    // Apc3dVision::align_params_t params(modelPC, rawPC);
    Apc3dVision::align_params_t params(rawPC, modelPC);

    params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    params.input_cloud_leaf_size = leaf;
    params.is_downsample_input_cloud = true;
    params.target_cloud_leaf_size = leaf;
    params.is_downsample_target_cloud = true;
    params.transformationEpsilon = transformationEpsilon;
    params.stepSize = stepSize;
    params.resolution = resolution;
    params.maxIterations = maxIterations;
    params.verbose = true;

    apc_vis.align(&params);

    // Save the file
    isSuccess = apc_vis.save_pcd_file(outputModelFileName, params.output_cloud);

    return 0;
}
