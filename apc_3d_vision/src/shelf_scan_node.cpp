#include "shelf_scan_node.h"
#include "shelf_scan.h"

bool shelf_scan_callback(apc_3d_vision::shelf_scan_bin::Request &req,
                         apc_3d_vision::shelf_scan_bin::Response &res) {
    shelf_scanner shelf_scanner(req.move_group.data);

    bool success;

    try {
        if (req.set_scan_radius.data) shelf_scanner.setShelfRadius(req.scan_radius.data);
        shelf_scanner.generatePath();
        success = shelf_scanner.execute();

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        res.success.data = false;
        return false;
    }

    res.success.data = success;
    return true;
}

bool shelf_scan_bin_callback(apc_3d_vision::shelf_scan_bin::Request &req,
                             apc_3d_vision::shelf_scan_bin::Response &res) {
    ROS_INFO_STREAM("Scanning Bin " << req.bin_id.data);
    shelf_scanner shelf_scanner(req.move_group.data, req.bin_id.data);

    bool success;

    try {
        // ROS_INFO_STREAM("move group: " << req.move_group);
        // ROS_INFO_STREAM("bin id: " << req.bin_id);
        // ROS_INFO_STREAM("set scan radius: " << req.set_scan_radius);
        // ROS_INFO_STREAM("scan radius: " << req.set_radius);
        if (req.set_scan_radius.data) {
            shelf_scanner.setShelfRadius(req.scan_radius.data);
        }
        shelf_scanner.generatePath();
        success = shelf_scanner.execute();

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        res.success.data = false;
        return false;
    }

    res.success.data = success;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "shelf_scan_node");

    ros::NodeHandle nh_("~");

    ros::ServiceServer scan_service =
        nh_.advertiseService("/shelf_scan", &shelf_scan_callback);
    ros::ServiceServer scan_bin_service =
        nh_.advertiseService("/shelf_scan_bin", &shelf_scan_bin_callback);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO_STREAM("shelf_scan_node services started!");

    while (ros::ok()) {
    }

    return 0;
}
