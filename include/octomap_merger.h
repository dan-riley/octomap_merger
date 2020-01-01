#include <Eigen/SVD>
#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>
#include <cmath>
#include "octomap_merger/OctomapArray.h"

using std::cout;
using std::endl;
using namespace octomap;
using namespace octomath;

// convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

#define MAXITER 500

void tree2PointCloud(OcTree *tree, pcl::PointCloud<pcl::PointXYZ>& pclCloud);

bool pointInBBox(pcl::PointXYZ& point,
                 pcl::PointXYZ& bboxMin,
                 pcl::PointXYZ& bboxMax);

Eigen::Matrix4f getICPTransformation(
    pcl::PointCloud<pcl::PointXYZ>& cloud1,
    pcl::PointCloud<pcl::PointXYZ>& cloud2,
    Eigen::Matrix4f& tfEst,
    double mapRes);

void transformTree(OcTree *tree, Eigen::Matrix4f& transform);

void align_maps(OcTree *tree1, OcTree *tree2, point3d translation,
                double roll, double pitch, double yaw, double res);

void expandLevel(OcTreeBase<OcTreeNode> *base, std::vector<OcTreeNode *> *nodePtrs);

unsigned expandNodeMultiLevel(OcTreeBase<OcTreeNode> *base, OcTree *tree,
                      OcTreeNode *node, unsigned currentDepth, int levels);

int getNodeDepth(OcTree* tree, point3d& point, OcTreeNode* node);

void merge_maps(OcTreeBase<OcTreeNode> *base, OcTree *tree1, OcTree *tree2);

class OctomapMerger {
  public:
    // Constructor
    OctomapMerger(ros::NodeHandle* nodehandle);
    // Destructor
    ~OctomapMerger();
    // Callbacks
    void callback_myMap(const octomap_msgs::Octomap::ConstPtr& msg);
    void callback_neighborMaps(const octomap_merger::OctomapArrayConstPtr &msg);
    // Public Methods
    void merge();
    // Variables
    bool myMapNew;
    bool otherMapsNew;

  /* Private Variables and Methods */
  private:
    ros::NodeHandle nh_;

    octomap_msgs::Octomap myMap;
    octomap_merger::OctomapArray neighbors;
    OcTreeBase<OcTreeNode> *base;
    octomap::OcTree *tree1;
    octomap::OcTree *tree2;

    ros::Subscriber sub_mymap;
    ros::Subscriber sub_neighbors;

    ros::Publisher pub_merged;

    void initializeSubscribers();
    void initializePublishers();
};
