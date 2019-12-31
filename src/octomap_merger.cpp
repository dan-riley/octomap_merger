#include <octomap_merger.h>

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
  if (argc < 4) {
    cout << "\nUsage: octomap_merger <input_file_1> <input_file_2> <output_file> [align]";
    cout << "\n                      ";
    cout << "[<translation_x> <translation_y> <translation_z>]";
    cout << "\n                      ";
    cout << "[<roll> <pitch> <yaw>]";
    cout << "\n\nInput files can be binary or full, but both must the same.\n";
    cout << "Output can be either, but does not have to be the same as input.\n";
    cout << "Use .bt or .ot file extension to specify\n\n";
    cout << "Add 'align' option to run ICP alignment on maps before merging.\n\n";
    exit(0);
  }

  std::string filename1 = std::string(argv[1]);
  std::string filename2 = std::string(argv[2]);
  std::string outputFilename = std::string(argv[3]);

  cout << "\nReading octree files...\n";

  OcTree *tree1, *tree2;
  if (filename1.substr(filename1.length() - 2) == "ot") {
    tree1 = dynamic_cast<OcTree*>(OcTree::read(filename1));
    tree2 = dynamic_cast<OcTree*>(OcTree::read(filename2));
  } else {
    tree1 = new OcTree(filename1);
    tree2 = new OcTree(filename2);
  }

  // Assume the resolution of each map is the same
  double res = tree1->getResolution();

  // Create this Base to allow newer implmentation of Octomap from the original
  OcTreeBase<OcTreeNode> *base = new typename OcTreeBase<OcTreeNode>::OcTreeBase(res);

  // Align the maps if desired.  Will slow down the process!
  if ((argc > 4) && (std::string(argv[4]) == "align")) {
    cout << "Registering map to Improve TF Estimate" << endl << endl;

    double roll, pitch, yaw;

    point3d translation;
    if (argc == 8 || argc == 11) {
      translation = point3d(atof(argv[5]), atof(argv[6]), atof(argv[7]));
    }
    if (argc == 11) {
      roll = atof(argv[8]);
      pitch = atof(argv[9]);
      yaw = atof(argv[10]);
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

    // initial TF Matrix
    cout << transform << endl;

    // make point clouds from each map
    pcl::PointCloud<pcl::PointXYZ> tree1Points;
    tree2PointCloud(tree1, tree1Points);
    pcl::PointCloud<pcl::PointXYZ> tree2Points;
    tree2PointCloud(tree2, tree2Points);

    // get refined matrix
    transform = getICPTransformation(tree1Points, tree2Points, transform, res);

    // Resulting transform after correction
    cout << transform << endl;

    if (roll != 0 ||
        pitch != 0 ||
        yaw != 0 ||
        translation.x() != 0 ||
        translation.y() != 0 ||
        translation.z() != 0 ) {
      transformTree(tree2, transform);
    }
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

  cout << "Compressing merged result\n";
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
