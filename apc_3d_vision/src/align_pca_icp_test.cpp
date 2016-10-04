/*
Copyright 2016 Australian Centre for Robotic Vision
*/

#include <apc_3d_vision.hpp>

#include <string>

int main(int argc, char** argv) {
    Apc3dVision apc_vis;

    // ICP Params
    double leaf = 0.005;

    // Smoothing Params
    // 3cm
    double smoothingRadius = 0.03;

    // Statistical outlier removal params
    // 10 nearest points
    double outlierNumberOfSamples = 10;
    // any points passed 2 std devs of query point are removed
    double stdDevMulThresh = 2;

    leaf = std::atof(argv[4]);
    smoothingRadius = std::atof(argv[5]);
    outlierNumberOfSamples = std::atof(argv[6]);
    stdDevMulThresh = std::atof(argv[7]);

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
    isSuccess = apc_vis.save_pcd_file(
        "raw_1.pcd", rawPC);

    // Remove statistical outliers
    apc_vis.outlier_removal(
        rawPC, rawPC, outlierNumberOfSamples, stdDevMulThresh);
    isSuccess = apc_vis.save_pcd_file(
        "raw_2.pcd", rawPC);

    // Smooth the raw point cloud
    apc_vis.smooth_cloud(rawPC, rawPCNormal, smoothingRadius);

    // Replace rawPC with rawPCNormal points
    pcl::copyPointCloud(*rawPCNormal, *rawPC);
    isSuccess = apc_vis.save_pcd_file(
        "raw_3.pcd", rawPC);
    //
    // isSuccess = apc_vis.save_pcd_file("model_demeaned.pcd", modelPC);
    // isSuccess = apc_vis.save_pcd_file("raw_demeaned.pcd", rawPC);

    // PERFORM PCA
    // I've found that performance is much worse when setting the input to be
    // the model and the target to be the raw cloud. I guess it makes sense that
    // you should fit the raw points (source) to the model (target) though.
    Apc3dVision::pca_params_t input_pca_params(rawPC);
    // Apc3dVision::pca_params_t input_pca_params(modelPC);  // Much slower
    input_pca_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    input_pca_params.verbose = true;

    apc_vis.align_pca(&input_pca_params);

    Apc3dVision::pca_params_t target_pca_params(modelPC);
    // Apc3dVision::pca_params_t target_pca_params(rawPC);  // Much slower
    target_pca_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    target_pca_params.verbose = true;

    apc_vis.align_pca(&target_pca_params);

    std::cout << "Finished PCA Alignment" << std::endl;

    // PERFORM ICP
    Apc3dVision::icp_params_t icp_params(
        input_pca_params.output_cloud, target_pca_params.output_cloud);

    icp_params.output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    icp_params.input_cloud_leaf_size = leaf;
    icp_params.is_downsample_input_cloud = true;
    icp_params.target_cloud_leaf_size = leaf;
    icp_params.is_downsample_target_cloud = true;
    icp_params.verbose = true;

    apc_vis.align_icp(&icp_params);

    // Transform both point clouds back into raw point cloud's frame
    // First undo transform from ICP
    // Do the inverse of the found transform on the target object
    // pcl::transformPointCloud(
    //     *(input_pca_params.output_cloud),
    //     *(input_pca_params.output_cloud),
    //     icp_params.transform.inverse());
    pcl::transformPointCloud(
        *(target_pca_params.output_cloud),
        *(target_pca_params.output_cloud),
        icp_params.transform.inverse());

    // Then undo transform from PCA
    // PCA transformed both input and target so both need to be undone
    Eigen::Matrix<double, 4, 4>
        transformation(Eigen::Matrix<double, 4, 4>::Identity());
    transformation.template block<3, 3>(0, 0) = input_pca_params.eigenvectors;
    // Set both models to the centroid of the input (raw cloud)
    transformation.template block<3, 1>(0, 3) =
        input_pca_params.centroid.template block<3, 1>(0, 0);
    pcl::transformPointCloud(
        *(input_pca_params.output_cloud),
        *(input_pca_params.output_cloud),
        transformation);
    pcl::transformPointCloud(
        *(target_pca_params.output_cloud),
        *(target_pca_params.output_cloud),
        transformation);

    // Visualisation
    pcl::visualization::PCLVisualizer visu1("Final Alignment");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        input_colour(target_pca_params.output_cloud, 0.0, 255.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_colour(input_pca_params.output_cloud, 255.0, 0.0, 0.0);
    visu1.addPointCloud(target_pca_params.output_cloud, input_colour, "model");
    visu1.addPointCloud(input_pca_params.output_cloud, output_colour, "raw");
    visu1.addCoordinateSystem(0.1);
    visu1.spin();

    // Save the file
    // isSuccess = apc_vis.save_pcd_file(
    //     outputModelFileName, icp_params.output_cloud);
    isSuccess = apc_vis.save_pcd_file(
        outputModelFileName, target_pca_params.output_cloud);
    isSuccess = apc_vis.save_pcd_file(
        "raw_4.pcd", input_pca_params.output_cloud);

    return 0;
}
