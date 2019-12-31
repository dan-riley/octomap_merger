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

void merge_maps(OcTreeBase<OcTreeNode> *base, OcTree *tree1, OcTree *tree2) {
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
}
