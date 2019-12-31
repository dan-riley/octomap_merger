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

void tree2PointCloud(OcTree *tree, pcl::PointCloud<pcl::PointXYZ>& pclCloud) {
  // now, traverse all leafs in the tree:
  for (OcTree::leaf_iterator it = tree->begin_leafs(),
       end = tree->end_leafs(); it != end; ++it)
  {
    if (tree->isNodeOccupied(*it)) {
      pclCloud.push_back(
          pcl::PointXYZ(it.getX(),
            it.getY(),
            it.getZ()
            )
          );
    }
  }
}

bool pointInBBox(pcl::PointXYZ& point,
                 pcl::PointXYZ& bboxMin,
                 pcl::PointXYZ& bboxMax) {

  return (point.x < bboxMax.x && point.x > bboxMin.x) &&
         (point.y < bboxMax.y && point.y > bboxMin.y) &&
         (point.z < bboxMax.z && point.z > bboxMin.z);
}

Eigen::Matrix4f getICPTransformation(
    pcl::PointCloud<pcl::PointXYZ>& cloud1,
    pcl::PointCloud<pcl::PointXYZ>& cloud2,
    Eigen::Matrix4f& tfEst,
    double mapRes) {

  // apply the tfEst to cloud2
  pcl::transformPointCloud(cloud2, cloud2, tfEst);

  // get the bounding region of cloud1 to
  // extract the points from cloud2 contained in the region
  pcl::PointXYZ minCloud1; pcl::PointXYZ maxCloud1;
  pcl::getMinMax3D(cloud1, minCloud1, maxCloud1);

  // filter out the points in cloud2 that are not in cloud1’s range
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud2.begin();
      it != cloud2.end(); it++) {

    if (pointInBBox(*it, minCloud1, maxCloud1)) {
      cloud2filtered->push_back(*it);
    }
  }

  // filter out the points in cloud1 that are not in cloud2’s range
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  // same for other cloud
  pcl::PointXYZ minCloud2filtered; pcl::PointXYZ maxCloud2filtered;
  pcl::getMinMax3D(*cloud2filtered, minCloud2filtered, maxCloud2filtered);

  minCloud2filtered = pcl::PointXYZ(
      minCloud2filtered.x - 1,
      minCloud2filtered.y - 1,
      minCloud2filtered.z - 1
      );

  maxCloud2filtered = pcl::PointXYZ(
      maxCloud2filtered.x + 1,
      maxCloud2filtered.y + 1,
      maxCloud2filtered.z + 1
      );

  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud1.begin();
      it != cloud1.end(); it++) {

    if (pointInBBox(*it, minCloud2filtered, maxCloud2filtered)) {
      cloud1filtered->push_back(*it);
    }
  }

  // Downsample for consistency and speed
  PointCloud::Ptr src(new PointCloud);
  PointCloud::Ptr tgt(new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize(10 * mapRes, 10 * mapRes, 10 * mapRes);
  grid.setInputCloud(cloud1filtered);
  grid.filter(*tgt);

  grid.setInputCloud(cloud2filtered);
  grid.filter(*src);

  // Align
  pcl::IterativeClosestPointNonLinear<PointT, PointT> reg;
  reg.setTransformationEpsilon(mapRes / 60);
  // Increased this from 10 to 200 because of errors when running
  // Not sure what the right numbers should be
  reg.setMaxCorrespondenceDistance(200 * mapRes);

  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
  PointCloud::Ptr reg_result;

  if (src->size() < tgt->size()) {
    reg.setInputSource(src);
    reg.setInputTarget(tgt);

    // Run the same optimization in a loop and visualize the results
    reg_result = src;
    reg.setMaximumIterations(2);
    for (int i=0; i < MAXITER; ++i) {
      // save cloud for visualization purpose
      src = reg_result;

      // Estimate
      reg.setInputSource(src);
      reg.align(*reg_result);

      // accumulate transformation between each Iteration
      Ti = reg.getFinalTransformation() * Ti;

      // if the difference between this transformation and the previous one
      // is smaller than the threshold, refine the process by reducing
      // the maximal correspondence distance
      if (reg.getMaxCorrespondenceDistance() > 0.2) {
        if (fabs((reg.getLastIncrementalTransformation().sum() - prev.sum()))
            < reg.getTransformationEpsilon())
          reg.setMaxCorrespondenceDistance(
              reg.getMaxCorrespondenceDistance() - 0.1
              );
      } else if (reg.getMaxCorrespondenceDistance() > 0.002) {
        if (fabs((reg.getLastIncrementalTransformation().sum() - prev.sum()))
            < reg.getTransformationEpsilon())
          reg.setMaxCorrespondenceDistance(
              reg.getMaxCorrespondenceDistance() - 0.001
              );
      }

      prev = reg.getLastIncrementalTransformation();
    }
  } else {
    reg.setInputSource(tgt);
    reg.setInputTarget(src);

    // Run the same optimization in a loop and visualize the results
    reg_result = tgt;
    reg.setMaximumIterations(2);
    for (int i=0; i < MAXITER; ++i) {
      // save cloud for visualization purpose
      tgt = reg_result;

      // Estimate
      reg.setInputSource(tgt);
      reg.align(*reg_result);

      // accumulate transformation between each Iteration
      Ti = reg.getFinalTransformation() * Ti;

      // if the difference between this transformation and the previous one
      // is smaller than the threshold, refine the process by reducing
      // the maximal correspondence distance
      if (reg.getMaxCorrespondenceDistance() > 0.2) {
        if (fabs((reg.getLastIncrementalTransformation().sum() - prev.sum()))
            < reg.getTransformationEpsilon())
          reg.setMaxCorrespondenceDistance(
              reg.getMaxCorrespondenceDistance() - 0.1
              );
      } else if (reg.getMaxCorrespondenceDistance() > 0.002) {
        if (fabs((reg.getLastIncrementalTransformation().sum() - prev.sum()))
            < reg.getTransformationEpsilon())
          reg.setMaxCorrespondenceDistance(
              reg.getMaxCorrespondenceDistance() - 0.001
              );
      }

      prev = reg.getLastIncrementalTransformation();
    }
    Ti = Ti.inverse();
  }

  return Ti * tfEst;
}

double getSign(double x) {
  if (x < 0) return -1;
  else return 1;
}

void transformTree(OcTree *tree, Eigen::Matrix4f& transform) {
  double treeRes = tree->getResolution();
  OcTree* transformed = new OcTree(treeRes);

  // build inverse transform
  Eigen::Matrix3f rotation;
  Eigen::Matrix3f invRotation;
  Eigen::Matrix4f invTransform;
  rotation << transform(0, 0), transform(0, 1), transform(0, 2),
              transform(1, 0), transform(1, 1), transform(1, 2),
              transform(2, 0), transform(0, 2), transform(2,  2);
  invRotation = rotation.transpose();
  invTransform <<
    invRotation(0, 0), invRotation(0, 1), invRotation(0, 2), -transform(0, 3),
    invRotation(1, 0), invRotation(1, 1), invRotation(1, 2), -transform( 1, 3),
    invRotation(2, 0), invRotation(2, 1), invRotation(2, 2), -transform( 2, 3),
    0, 0, 0, 1;

  // size in each coordinate of each axis.
  double minX, maxX, minY, maxY, minZ, maxZ;

  // get the minimum and maxin y so we can step along each row
  tree->getMetricMin(minX, minY, minZ);
  tree->getMetricMax(maxX, maxY, maxZ);

  // allocate a vector of points
  std::vector<point3d> points;

  // make 8 points to make a map bounding box, performing the tf on them
  // to get the range of values in the transformed map
  points.push_back(point3d(maxX,minY,minZ));
  points.push_back(point3d(minX,minY,minZ));
  points.push_back(point3d(minX,maxY,minZ));
  points.push_back(point3d(maxX,maxY,minZ));
  points.push_back(point3d(maxX,minY,maxZ));
  points.push_back(point3d(minX,minY,maxZ));
  points.push_back(point3d(minX,maxY,maxZ));
  points.push_back(point3d(maxX,maxY,maxZ));

  // transform the points
  for (unsigned i= 0; i < points.size(); i++) {
    Eigen::Vector4f point(points[i].x(), points[i].y(), points[i].z(), 1);
    point = transform * point;
    points[i] = point3d(point(0), point(1), point(2));
  }

  // go through tf’d points to get a newb box
  minX = points[0].x(); minY = points[0].y(); minZ = points[0].z();
  maxX = points[0].x(); maxY = points[0].y(); maxZ = points[0].z();
  for (unsigned i=0; i < points.size(); i++) {
    minX = (points[i].x() < minX) ? points[i].x() : minX;
    minY = (points[i].y() < minY) ? points[i].y() : minY;
    minZ = (points[i].z() < minZ) ? points[i].z() : minZ;
    maxX = (points[i].x() > maxX) ? points[i].x() : maxX;
    maxY = (points[i].y() > maxY) ? points[i].y() : maxY;
    maxZ = (points[i].z() > maxZ) ? points[i].z() : maxZ;
  }

  // go through the possible destination voxels on a row by row basis
  // and calculate occupancy from source voxels with inverse tf
  for (double z = minZ - treeRes / 2; z < (maxZ + treeRes / 2); z += treeRes) {
    for (double y = minY - treeRes / 2; y < (maxY + treeRes / 2); y += treeRes) {
      for (double x = minX - treeRes / 2; x < (maxX + treeRes / 2); x += treeRes) {
        OcTreeKey destVoxel = transformed->coordToKey(
            point3d(x,y,z)
            );
        Eigen::Vector4f point(x,y,z,1);
        point = invTransform * point;
        point3d sourcePoint = point3d(point(0), point(1), point(2));
        OcTreeKey sourceVoxel = tree->coordToKey(sourcePoint);
        point3d nn = tree->keyToCoord(sourceVoxel);

        // use nearest neighbour to set new occupancy in the transformed map
        OcTreeNode *oldNode = tree->search(sourceVoxel);

        // Occupancies to interpolate between
        double c000, c001, c010, c011, c100, c101, c110,
               c111, c00, c01, c10, c11, c0, c1;
        double xd,yd,zd;

        // differences in each direction between next closest voxel
        xd = (sourcePoint.x() - nn.x()) / treeRes;
        yd = (sourcePoint.y() - nn.y()) / treeRes;
        zd = (sourcePoint.z() - nn.z()) / treeRes;

        if (oldNode != NULL) {
          c000 = oldNode->getOccupancy();
          OcTreeNode *node;

          // c001
          if ((node = tree->search(
                  point3d(nn.x(), nn.y(), nn.z() +
                    getSign(zd) * treeRes)))
              != NULL) {
            c001 = node->getOccupancy();
          } else
            c001 = 0;

          // c010
          if ((node=tree->search(
                  point3d(nn.x(),
                    nn.y() + getSign(yd) * treeRes,
                    nn.z())))
              != NULL) {
            c010 =node->getOccupancy();
          } else
            c010 = 0;

          // c011
          if ((node=tree->search(
                  point3d(nn.x(),
                    nn.y() + getSign(yd) * treeRes,
                    nn.z() + getSign(zd) * treeRes)))
              != NULL) {
            c011 = node->getOccupancy();
          } else
            c011 = 0;

          // c100
          if ((node=tree->search(
                  point3d(nn.x() + getSign(xd) * treeRes,
                    nn.y(),
                    nn.z())))
              != NULL) {
            c100 = node->getOccupancy();
          } else
            c100 = 0;

          // c101
          if ((node=tree->search(
                  point3d(nn.x() + getSign(xd) * treeRes,
                    nn.y(),
                    nn.z() + getSign(zd) * treeRes)))
              != NULL) {
            c101 = node->getOccupancy();
          } else
            c101 = 0;

          // c110
          if ((node=tree->search(
                  point3d(nn.x() + getSign(xd) * treeRes,
                    nn.y() + getSign(yd) * treeRes,
                    nn.z())))
              != NULL) {
            c110 = node->getOccupancy();
          } else
            c110 = 0;

          // c111
          if ((node=tree->search(
                  point3d(nn.x() + getSign(xd) * treeRes,
                    nn.y() + getSign(yd) * treeRes,
                    nn.z() + getSign(zd) * treeRes)))
              != NULL) {
            c111 = node->getOccupancy();
          } else
            c111 = 0;

          // Interpolate in x
          c00 = (1-fabs(xd)) * c000 + fabs(xd) * c100;
          c10 = (1-fabs(xd)) * c010 + fabs(xd) * c110;
          c01 = (1-fabs(xd)) * c001 + fabs(xd) * c101;
          c11 = (1-fabs(xd)) * c011 + fabs(xd) * c111;

          // interpolate in y
          c0 = (1-fabs(yd)) * c00 + fabs(yd) * c10;
          c1 = (1-fabs(yd)) * c01 + fabs(yd) * c11;

          // now let’s assign the new node value
          OcTreeNode *newNode = transformed->updateNode(destVoxel, true);
          newNode->setLogOdds(logodds((1 - fabs(zd)) * c0 + fabs(zd) * c1));
        }
      }
    }
  }

  tree->swapContent(*transformed);

  delete transformed;
}

void expandLevel(OcTreeBase<OcTreeNode> *base, std::vector<OcTreeNode *> *nodePtrs) {
  unsigned size = nodePtrs->size();
  for (unsigned i=0; i < size; i++) {
    OcTreeNode *parent = nodePtrs->front();
    base->expandNode(parent);
    nodePtrs->erase(nodePtrs->begin());
    for (unsigned j=0; j < 8; j++) {
      nodePtrs->push_back(base->getNodeChild(parent, j));
    }
  }
}

unsigned expandNodeMultiLevel(OcTreeBase<OcTreeNode> *base, OcTree *tree,
                      OcTreeNode *node, unsigned currentDepth, int levels) {
  if (currentDepth == (int)tree->getTreeDepth()) {
    return 0;
  }

  int levelsCounter = 0;
  std::vector<OcTreeNode *> nodePtrs;
  nodePtrs.push_back(node);

  for (unsigned i=0; i < levels; i++) {
    if (currentDepth == (int)tree->getTreeDepth()) {
      return levelsCounter;
    }
    expandLevel(base, &nodePtrs);
    levelsCounter++;
    currentDepth++;
  }

  return levelsCounter;
}

/*
 * Searches for a node at a given point
 * and returns the depth in the tree of that node
 * Assumes you have called search before and
 * know its actually there.
 * Returns -1 if it couldn’t find anything
 */

int getNodeDepth(OcTree* tree, point3d& point, OcTreeNode* node) {
  for (int depth = tree->getTreeDepth(); depth > 1; depth--) {
    if (tree->search(point, depth) == node)
      return depth;
  }

  return -1;
}

int main(int argc, char** argv) {
  std::string filename1 = std::string(argv[1]);
  std::string filename2 = std::string(argv[2]);
  std::string outputFilename = std::string(argv[3]);

  cout << "\nReading octree files...\n";

  double roll, pitch, yaw;

  point3d translation;
  if (argc == 7 || argc == 10) {
    translation = point3d(atof(argv[4]), atof(argv[5]), atof(argv[6]));
  }
  if (argc == 10) {
    roll = atof(argv[7]);
    pitch = atof(argv[8]);
    yaw = atof(argv[9]);
  } else {
    roll = 0;
    pitch = 0;
    yaw = 0;
  }

  Pose6D pose(translation.x(),
      translation.y(),
      translation.z(),
      roll, pitch, yaw);

  // build a transform matrix
  Eigen::Matrix4f transform;
  std::vector<double> coeffs;
  pose.rot().toRotMatrix(coeffs);

  transform << coeffs[0], coeffs[1], coeffs[2], translation.x(),
               coeffs[3], coeffs[4], coeffs[5], translation.y(),
               coeffs[6], coeffs[7], coeffs[8], translation.z(),
               0, 0, 0, 1;

  OcTree *tree1, *tree2;
  if (filename1.substr(filename1.length() - 2) == "ot") {
    tree1 = dynamic_cast<OcTree*>(OcTree::read(filename1));
    tree2 = dynamic_cast<OcTree*>(OcTree::read(filename2));
  } else {
    tree1 = new OcTree(filename1);
    tree2 = new OcTree(filename2);
  }

  // initial TF Matrix
  std::cout << transform << std::endl;

  cout << "Registering map to Improve TF Estimate" << endl << endl;

  // make point clouds from each map
  pcl::PointCloud<pcl::PointXYZ> tree1Points;
  tree2PointCloud(tree1, tree1Points);
  pcl::PointCloud<pcl::PointXYZ> tree2Points;
  tree2PointCloud(tree2, tree2Points);

  double res = tree1->getResolution();
  // get refined matrix
  transform = getICPTransformation(tree1Points, tree2Points, transform, res);

  // Create this Base to allow newer implmentation of Octomap from the original
  OcTreeBase<OcTreeNode> *base = new typename OcTreeBase<OcTreeNode>::OcTreeBase(res);

  if (roll != 0 ||
      pitch != 0 ||
      yaw != 0 ||
      translation.x() != 0 ||
      translation.y() != 0 ||
      translation.z() != 0 ) {
    transformTree(tree2 , transform);
  }

  // begin merging algorithm
  // traverse nodes in tree2 to add them to tree1
  for (OcTree::leaf_iterator it = tree2->begin_leafs();
      it != tree2->end_leafs(); ++it) {
    if(tree2->isNodeOccupied(*it)) {
      it->setLogOdds(logodds(0.6));
    }

    // find if the current node maps a point in map1
    OcTreeNode *nodeIn1 = tree1->search(it.getCoordinate());
    OcTreeKey nodeKey=tree1->coordToKey(it.getCoordinate());
    point3d point = it.getCoordinate();
    if (nodeIn1 != NULL) {
      // get the depth of already mapped space in 1 and compare to 2
      int depthIn1 = getNodeDepth(tree1, point, nodeIn1);
      if (depthIn1 != -1) {
        int depthDiff = it.getDepth() - depthIn1;
        if (depthDiff == 0 ) {
          tree1->updateNode(nodeKey, it->getLogOdds() );
        } else if(depthDiff > 0) {
          // map2 is lower depth, add children to 1 if it’s not a leaf
          for (int i=0; i < depthDiff; i++) {
            if (depthIn1 == (int)tree1->getTreeDepth()) {
              break;
            }

            base->expandNode(nodeIn1);
            nodeKey = tree1->coordToKey(point);
            depthIn1++;
          }
          nodeIn1->setLogOdds(
              logodds(nodeIn1->getOccupancy() + it->getOccupancy()));
        } else if (depthDiff < 0) {
          // map1 is lower depth, add children to 2
          expandNodeMultiLevel(base, tree2, tree2->search(point),
              it.getDepth(), abs(depthDiff));
          // now that we are expanded the other expanded nodes
          // will be handled in subsequent loop iterations
        }
      }
    } else {
      OcTreeNode *newNode = tree1->updateNode(point, true);
      newNode->setLogOdds(it->getLogOdds());
    }
  }

  std::cout << "Compressing merged result\n";
  tree1->prune();
  // tree1 is now the compressed merged map

  // write merged map to file
  if (outputFilename.substr(outputFilename.length() - 2) == "ot") {
    tree1->write(outputFilename);
  } else {
    tree1->writeBinary(outputFilename);
  }

  delete tree1;
  delete tree2;
}
