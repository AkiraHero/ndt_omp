#include <iostream>
//#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "QStringList"
#include "pose3d.hpp"
#include <pclomp/ndt_omp.h>

// align point clouds and measure processing time
pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 pose2matrix(Pose3d<double> const&pose){
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 ma;
    ma.setIdentity();
    double gtyaw = pose.yaw;
    double gtx = pose.x;
    double gty = pose.y;
    ma(0, 0) = cos(gtyaw);
    ma(0, 1) = -sin(gtyaw);
    ma(1, 0) = sin(gtyaw);
    ma(1, 1) = cos(gtyaw);
    ma(0, 3) = gtx;
    ma(1, 3) = gty;
    return ma;
};


pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, Pose3d<double>guess=Pose3d<double>()) {
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

//  auto t1 = ros::WallTime::now();
  registration->align(*aligned, pose2matrix(guess));
//  auto t2 = ros::WallTime::now();
//  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
//
//  for(int i=0; i<10; i++) {
//    registration->align(*aligned);
//  }
//  auto t3 = ros::WallTime::now();
//  std::cout << "10times: " << (t3 - t2).toSec() * 1000 << "[msec]" << std::endl;
//  std::cout << "fitness: " << registration->getFitnessScore() << std::endl << std::endl;

  return aligned;
}
void loadguessfile(std::ifstream &fileStream, std::vector<Pose3d<double>>&poselist){
    char row[1000];
    poselist.clear();
    while(fileStream.getline(row, 1000)) {
        QStringList strList = QString(row).split(QString(','));
        Pose3d<double> pose;
        pose.timestamp = strList[0].toInt();
        pose.yaw = strList[3].toDouble();
        pose.x = strList[1].toDouble();
        pose.y = strList[2].toDouble();
        poselist.push_back(pose);
    }
}

int findguessvalue(int timestamp, std::vector<Pose3d<double>>const & poselist){
    auto func1 = [](int timestamp, Pose3d<double>a) -> bool {
        return a.timestamp > timestamp;
    };
    auto iterator = std::upper_bound(poselist.begin(), poselist.end(), timestamp, func1);
    if(iterator != poselist.end()){
        auto rtime = iterator->timestamp;
        auto deltatime = abs(rtime - timestamp);
        auto rtime2 = (iterator - 1)->timestamp;
        auto deltatime2 = abs(rtime2 - timestamp);
        if(deltatime < deltatime2)
            return  iterator - poselist.begin();
        else
            return  iterator - poselist.begin() - 1;
    }
    else{
        if(abs((iterator - 1)->timestamp - timestamp) < 30)
            return (poselist.end() - poselist.begin() - 1);
    }
    return -1;
}

int main(int argc, char** argv) {
//  if(argc != 3) {
//    std::cout << "usage: align target.pcd source.pcd" << std::endl;
//    return 0;
//  }
    std::string timestampfile = "/media/akira/OrangePassport/1121/jxl/dataset_tmp/2018-11-21-13-33-58_sampled.lms.csv";
    std::ifstream timefilestr;
    timefilestr.open(timestampfile);
    std::string pcfdir = "/media/akira/OrangePassport/1121/jxl/dataset_tmp/pcl-lms";
    std::ifstream posguessfile("/media/akira/OrangePassport/1121/jxl/dataset_tmp/2018-11-21-13-33-58_sampled_localgt.nav_mei");
    std::vector<Pose3d<double>> poselist;
    loadguessfile(posguessfile, poselist);
    int timestamp,lasttimestamp;
    timefilestr>>lasttimestamp;
    std::ofstream transformOut("/home/akira/poss-server/dataprocessing/campus1121/ndttrans.csv");
    std::ofstream transformOut2("/home/akira/poss-server/dataprocessing/campus1121/ndttrans_xyt.csv");
    int frame = 0;
    while (timefilestr>>timestamp){

        frame ++ ;
        char filename[100];
        //target
        sprintf(filename, "/media/akira/OrangePassport/1121/jxl/dataset_tmp/pcl-lms/%d.pcd", lasttimestamp);
        std::string target_pcd = filename;
        //source
        sprintf(filename, "/media/akira/OrangePassport/1121/jxl/dataset_tmp/pcl-lms/%d.pcd", timestamp);
        std::string source_pcd = filename;
        lasttimestamp = timestamp;
//
//        if(timestamp < 48895418)
//            continue;
        int validguessinx = findguessvalue(timestamp, poselist);
        Pose3d <double> guesspose;
        if(validguessinx != -1)
            guesspose = poselist[validguessinx];
        else{
            std::cout << "guesserror!" <<std::endl;
        }
//        guesspose = Pose3d<double>();

        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        if (pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
            std::cerr << "failed to load " << target_pcd << std::endl;
            return 0;
        }
        if (pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
            std::cerr << "failed to load " << source_pcd << std::endl;
            return 0;
        }
//


//        pcl::visualization::PCLVisualizer vis("vis");
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud, 0.0, 255.0, 0.0);
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned, 0.0, 0.0, 255.0);
//        vis.addPointCloud(target_cloud, target_handler, "target");
//        vis.addPointCloud(source_cloud, source_handler, "source");
////        vis.addPointCloud(aligned, aligned_handler, "aligned");
//        vis.spin();
//




        // downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

        voxelgrid.setInputCloud(target_cloud);
        voxelgrid.filter(*downsampled);
        *target_cloud = *downsampled;

        voxelgrid.setInputCloud(source_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;

//  ros::Time::init();

        // benchmark
//        std::cout << "--- pcl::NDT ---" << std::endl;
//        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
//                new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
//        ndt->setResolution(1.0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned ;//= align(ndt, target_cloud, source_cloud);





        std::vector<int> num_threads = {1, omp_get_max_threads()};
        std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
//                {"KDTREE",  pclomp::KDTREE}
                {"DIRECT7", pclomp::DIRECT7},
//                {"DIRECT1", pclomp::DIRECT1}
        };






        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(
                new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        ndt_omp->setResolution(5);

        for (int n : num_threads) {
            for (const auto &search_method : search_methods) {
//                std::cout << "--- pclomp::NDT (" << search_method.first << ", " << n << " threads) ---" << std::endl;
                ndt_omp->setNumThreads(n);
                ndt_omp->setNeighborhoodSearchMethod(search_method.second);
                aligned = align(ndt_omp, target_cloud, source_cloud, guesspose);
            }
        }
        auto trans = ndt_omp->getFinalTransformation();
//        trans = pose2matrix(guesspose);
        transformOut << timestamp << ","
                  << trans(0, 0) << ',' << trans(0, 1) << "," << trans(0, 2)<< "," << trans(0, 3) << ","
                << trans(1, 0) << ',' << trans(1, 1) << "," << trans(1, 2)<< "," << trans(1, 3) << ","
                << trans(2, 0) << ',' << trans(2, 1) << "," << trans(2, 2)<< "," << trans(2, 3) << ","
                << trans(3, 0) << ',' << trans(3, 1) << "," << trans(3, 2)<< "," << trans(3, 3)
                << std::endl;
        double x = trans(0, 3);
        double y = trans(1, 3);
        double theta = atan2(-trans(0,1), trans(0,0));
        transformOut2 << timestamp << ","
                     << trans(0, 3) << ","
                     << trans(1, 3) << ","
                     << atan2(-trans(0,1), trans(0,0))
                     << std::endl;
        std::cout << timestamp << std::endl;

//        pcl::PointCloud<pcl::PointXYZ>::Ptr gtcloud(new pcl::PointCloud<pcl::PointXYZ> ());
//        decltype(trans) gttrans;
//        gttrans.setIdentity();
//        1.03436,0.23394,0.19130
//        double gtx = 1.03436;
//        double gty = 0.23394;
//        double gtyaw= 0.19130;
//        gttrans(0, 0) = cos(gtyaw);
//        gttrans(0, 1) = -sin(gtyaw);
//        gttrans(1, 0) = sin(gtyaw);
//        gttrans(1, 1) = cos(gtyaw);
//        gttrans(0, 3) = gtx;
//        gttrans(1, 3) = gty;
//        pcl::transformPointCloud(*source_cloud, *gtcloud, pose2matrix(guesspose));

//
//        if(timestamp > 0){
//                    pcl::visualization::PCLVisualizer vis("vis");
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud, 0.0, 255.0, 0.0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned, 0.0, 0.0, 255.0);
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gtcloud_handler(gtcloud, 255.0, 255.0, 255.0);
//
//            vis.addPointCloud(target_cloud, target_handler, "target");
//        vis.addPointCloud(source_cloud, source_handler, "source");
//        vis.addPointCloud(aligned, aligned_handler, "aligned");
////            vis.addPointCloud(gtcloud, gtcloud_handler, "gt");
//
//            vis.spin();
//
//        }

    }
    return 0;
}
