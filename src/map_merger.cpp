#include <octomap_merger.h>

void merge_maps(OcTreeStamped *tree1, OcTree *tree2, bool full_merge,
                bool replace, bool overwrite, bool free_prioritize) {
  // full_merge = add node values instead of replace/overwrite
  // replace = always replace an existing node
  // overwrite = replace an existing node if the node is marked with timestamp = 0
  // free_prioritize = only available with full_merge; if any map shows free node, make free
  // Expand tree so we search all nodes
  tree2->expand();

  // Timestamp to mark a node as an original from the owner or not
  int ts = replace ? 1 : 0;

  // traverse nodes in tree2 to add them to tree1
  for (OcTree::leaf_iterator it = tree2->begin_leafs(); it != tree2->end_leafs(); ++it) {
    OcTreeKey nodeKey = it.getKey();
    OcTreeNodeStamped *nodeIn1 = tree1->search(nodeKey);
    if (nodeIn1 != NULL) {
      // Merge or replace the node in tree1 based on input parameters
      if (full_merge) {
        // If set, if any map shows a node free, make it free
        if (free_prioritize) {
          if ((nodeIn1->getOccupancy() < 0.5) || (it->getOccupancy() < 0.5))
            tree1->setNodeValue(nodeKey, octomap::logodds(0));
          else
            tree1->updateNode(nodeKey, it->getLogOdds());
        } else {
          tree1->updateNode(nodeKey, it->getLogOdds());
        }

        nodeIn1->setTimestamp(ts);
      } else if (replace || (overwrite && (nodeIn1->getTimestamp() == 0))) {
        tree1->setNodeValue(nodeKey, it->getLogOdds());
        nodeIn1->setTimestamp(ts);
      }
    } else {
      // Add the node to tree1
      OcTreeNodeStamped *newNode = tree1->setNodeValue(nodeKey, it->getLogOdds());
      newNode->setTimestamp(ts);
    }
  }
}

double build_diff_tree(OcTree *tree1, OcTree *tree2, OcTree *tree_diff) {
  // Find the differences in tree2 from tree1 and write to a new diff tree

  // Expand tree so we search all nodes
  tree2->expand();
  double num_new_nodes = 0;

  // traverse nodes in tree2 to add them to tree1
  for (OcTree::leaf_iterator it = tree2->begin_leafs(); it != tree2->end_leafs(); ++it) {
    OcTreeKey nodeKey = it.getKey();
    OcTreeNode *nodeIn1 = tree1->search(nodeKey);
    if (nodeIn1 != NULL) {
      // Change the diff tree (may be add, or may be new) if the two maps are different
      if (nodeIn1->getOccupancy() != it->getOccupancy()) {
        tree_diff->setNodeValue(nodeKey, it->getLogOdds());
      }
    } else {
      // Add to the diff tree if it's a new node
      OcTreeNode *newDiffNode = tree_diff->setNodeValue(nodeKey, it->getLogOdds());
      num_new_nodes++;
    }
  }

  return num_new_nodes;
}
