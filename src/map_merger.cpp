#include <octomap_merger.h>

double merge_maps(OcTree *tree1, OcTree *tree2, bool full_merge, bool free_prioritize) {
  double size = 0;
  // Expand tree2 so we search all nodes
  tree2->expand();

  // traverse nodes in tree2 to add them to tree1
  for (OcTree::leaf_iterator it = tree2->begin_leafs();
                             it != tree2->end_leafs(); ++it) {

    // Calculate the total volume of the tree so it can be saved
    size += it.getSize();

    // find if the current node maps a point in map1
    point3d point = it.getCoordinate();
    OcTreeNode *nodeIn1 = tree1->search(point);
    if (nodeIn1 != NULL) {
      // If not running in full merge mode, ignore nodes already in robot
      if (full_merge) {
        // Add the probability of tree2 node to the found node
        OcTreeKey nodeKey = tree1->coordToKey(point);
        // If set, if any map shows a node free, make it free
        if (free_prioritize) {
          OcTreeNode *newNode = tree1->updateNode(point, true);
          if ((newNode->getOccupancy() < 0.5) || (it->getOccupancy() < 0.5))
            newNode->setLogOdds(octomap::logodds(0));
          else
            tree1->updateNode(nodeKey, it->getLogOdds());
        } else {
          tree1->updateNode(nodeKey, it->getLogOdds());
        }
      }
    } else {
      // Create a new node and set the probability from tree2
      OcTreeNode *newNode = tree1->updateNode(point, true);
      newNode->setLogOdds(it->getLogOdds());
    }
  }

  return size;
}
