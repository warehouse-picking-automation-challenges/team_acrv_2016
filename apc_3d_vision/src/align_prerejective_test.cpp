/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <apc_3d_vision.hpp>

#include <string>

int main(int argc, char** argv) {
    Apc3dVision apc_vis;

    double leaf = 0.005;
    double normalRadiusSearch = 0.01;
    double featureRadiusSearch = 0.025;
    // Number of points to sample for generating/prerejecting a pose
    int numberOfSamples = 3;
    // Number of nearest features to use
    int correspondenceRandomness = 5;
    // Polygonal edge length similarity threshold
    double similarityThreshold = 0.9;
    // Inlier threshold
    double maxCorrespondenceDistance = 2.5*leaf;
    // Required inlier fraction for accepting a pose hypothesis
    double inlierFraction = 0.25;
    // 3cm
    double smoothingRadius = 0.03;
    // 10 nearest points
    double outlierNumberOfSamples = 10;
    // any points passed 2 std devs of query point are removed
    double stdDevMulThresh = 2;
    int maxIterations = 5000;

    leaf = std::atof(argv[4]);
    normalRadiusSearch = std::atof(argv[5]);
    featureRadiusSearch = std::atof(argv[6]);
    numberOfSamples = std::atof(argv[7]);
    correspondenceRandomness = std::atof(argv[8]);
    similarityThreshold = std::atof(argv[9]);
    maxCorrespondenceDistance = std::atof(argv[10]);
    inlierFraction = std::atof(argv[11]);
    smoothingRadius = std::atof(argv[12]);
    outlierNumberOfSamples = std::atof(argv[13]);
    stdDevMulThresh = std::atof(argv[14]);
    maxIterations = std::atof(argv[15]);

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

    // // Calculate the centroid
    // Eigen::Vector4f modelCentroid;
    // Eigen::Vector4f rawCentroid;
    // pcl::compute3DCentroid(*modelPC, modelCentroid);
    // pcl::compute3DCentroid(*rawPC, rawCentroid);
    //
    // // Subtract the centroid from the point cloud
    // pcl::demeanPointCloud<pcl::PointXYZ> (
    //     *modelPC, modelCentroid, *modelPC);
    // pcl::demeanPointCloud<pcl::PointXYZ> (
    //     *rawPC, rawCentroid, *rawPC);

    // Remove statistical outliers
    apc_vis.outlier_removal(
        rawPC, rawPC, outlierNumberOfSamples, stdDevMulThresh);

    // Smooth the raw point cloud
    apc_vis.smooth_cloud(rawPC, rawPCNormal, smoothingRadius);
    // Replace rawPC with rawPCNormal points
    pcl::copyPointCloud(*rawPCNormal, *rawPC);

    // Downsample the raw point cloud
    // down_sample(rawPC, rawPC, leafSize);

    isSuccess = apc_vis.save_pcd_file("model_demeaned.pcd", modelPC);
    isSuccess = apc_vis.save_pcd_file("raw_demeaned.pcd", rawPC);

    // input, target, output

    Apc3dVision::align_prerejective_params_t params;

    params.input_cloud = modelPC;
    params.target_cloud = rawPC;
    params.output_cloud = registeredPC;
    params.leaf_size = leaf;
    params.normalRadiusSearch = normalRadiusSearch;
    params.featureRadiusSearch = featureRadiusSearch;
    params.maxIterations = maxIterations;
    params.numberOfSamples = numberOfSamples;
    params.correspondenceRandomness = correspondenceRandomness;
    params.similarityThreshold = similarityThreshold;
    params.maxCorrespondenceDistance = maxCorrespondenceDistance;
    params.inlierFraction = inlierFraction;
    params.verbose = true;

    apc_vis.align_prerejective(params);

    // Save the file
    isSuccess = apc_vis.save_pcd_file(outputModelFileName, registeredPC);

    return 0;
}
