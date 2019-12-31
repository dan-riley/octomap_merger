#include <Eigen/SVD>
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
#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>
#include <cmath>

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

void expandLevel(OcTreeBase<OcTreeNode> *base, std::vector<OcTreeNode *> *nodePtrs);

unsigned expandNodeMultiLevel(OcTreeBase<OcTreeNode> *base, OcTree *tree,
                      OcTreeNode *node, unsigned currentDepth, int levels);

int getNodeDepth(OcTree* tree, point3d& point, OcTreeNode* node);
