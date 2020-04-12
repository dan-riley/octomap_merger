#include <octomap_merger.h>

OctomapMerger::OctomapMerger(ros::NodeHandle* nodehandle):nh_(*nodehandle) {
    ROS_INFO("Constructing OctomapMerger Class");

    std::string nn = ros::this_node::getName();
    // Load parameters from launch file
    nh_.param<std::string>(nn + "/vehicle", id, "H01");
    // Type of agent (robot or base)
    nh_.param<std::string>(nn + "/type", type, "robot");
    // Full merging or prioritize own map
    nh_.param(nn + "/fullMerge", full_merge, false);
    // Keep free space from any agent instead of adding obstacles
    nh_.param(nn + "/freePrioritize", free_prioritize, false);
    // Whether to publish PCL of merged map
    nh_.param(nn + "/publishMergedPCL", publish_merged_pcl, false);
    // Whether to publish PCL of diff map
    nh_.param(nn + "/publishDiffPCL", publish_diff_pcl, false);
    // Octomap type: 0: Binary, 1: Full
    nh_.param(nn + "/octoType", octo_type, 0);
    // Map resolution
    nh_.param(nn + "/resolution", resolution, (double)0.2);
    // Map size threshold to trigger a map merge
    nh_.param(nn + "/mapThresh", map_thresh, 50);

    // Topics for Subscribing and Publishing
    nh_.param<std::string>(nn + "/mapTopic", map_topic, "octomap_binary");
    nh_.param<std::string>(nn + "/neighborsTopic", neighbors_topic, "neighbor_maps");
    nh_.param<std::string>(nn + "/mergedTopic", merged_topic, "merged_map");
    nh_.param<std::string>(nn + "/mapDiffsTopic", map_diffs_topic, "map_diffs");
    nh_.param<std::string>(nn + "/numDiffsTopic", num_diffs_topic, "numDiffs");
    nh_.param<std::string>(nn + "/pclMergedTopic", pcl_merged_topic, "pc2_merged");
    nh_.param<std::string>(nn + "/pclDiffTopic", pcl_diff_topic, "pc2_diff");

    initializeSubscribers();
    initializePublishers();
    myMapNew = false;
    otherMapsNew = false;

    // Initialize Octomap holders once, assign/overwrite each loop
    tree_merged = new octomap::OcTreeStamped(resolution);
    tree_sys = new octomap::OcTree(resolution);
    tree_old = new octomap::OcTree(resolution);
    tree_temp = new octomap::OcTree(resolution);
    tree_diff = new octomap::OcTree(resolution);
    num_diffs = 0;

    if (publish_diff_pcl) tree_pcl = new octomap::OcTreeStamped(resolution);
}

// Destructor
OctomapMerger::~OctomapMerger() {
}

void OctomapMerger::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    sub_mymap = nh_.subscribe(map_topic, 100,
                              &OctomapMerger::callback_myMap, this);
    sub_neighbors = nh_.subscribe(neighbors_topic, 100,
                                  &OctomapMerger::callback_neighborMaps, this);
}

void OctomapMerger::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    pub_merged = nh_.advertise<octomap_msgs::Octomap>(merged_topic, 1, true);
    pub_size = nh_.advertise<std_msgs::UInt32>(num_diffs_topic, 1, true);
    pub_mapdiffs = nh_.advertise<octomap_merger::OctomapArray>(map_diffs_topic, 1, true);
    if (publish_merged_pcl)
        pub_merged_pcl = nh_.advertise<sensor_msgs::PointCloud2>(pcl_merged_topic, 1, true);
    if (publish_diff_pcl)
        pub_diff_pcl = nh_.advertise<sensor_msgs::PointCloud2>(pcl_diff_topic, 1, true);
}

// Callbacks
void OctomapMerger::callback_myMap(const octomap_msgs::OctomapConstPtr& msg) {
  myMap = *msg;
  myMapNew = true;
}

void OctomapMerger::callback_neighborMaps(
                const octomap_merger::OctomapNeighborsConstPtr& msg) {
  neighbors = *msg;
  otherMapsNew = true;
}

sensor_msgs::PointCloud2 OctomapMerger::buildPCL(OcTreeStamped *tree) {
  tree->expand();
  sensor_msgs::PointCloud2 pcl;
  PointCloud::Ptr occupiedCells(new PointCloud);
  tree2PointCloud(tree, *occupiedCells);
  pcl::toROSMsg(*occupiedCells, pcl);
  pcl.header.stamp = ros::Time::now();
  pcl.header.frame_id = "world";

  return pcl;
}

void OctomapMerger::merge() {
  double num_nodes = 0;
  if (type == "robot") {
    if (octo_type == 0)
      tree_sys = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(myMap);
    else
      tree_sys = (octomap::OcTree*)octomap_msgs::fullMsgToMap(myMap);

    if (!tree_sys) return;

    // Get the diff tree from the current robot map and the last one saved
    num_nodes = build_diff_tree(tree_old, tree_sys, tree_diff);
  }

  octomap_msgs::Octomap msg;

  // If there are enough new nodes, save the robot map for next iter, and merge differences
  if (num_nodes > map_thresh) {
    tree_old->swapContent(*tree_sys);
    merge_maps(tree_merged, tree_diff, full_merge, true, false, free_prioritize);

    // Publish the diffs octomap
    tree_diff->prune();
    num_diffs++;
    if (octo_type == 0)
      octomap_msgs::binaryMapToMsg(*tree_diff, msg);
    else
      octomap_msgs::fullMapToMsg(*tree_diff, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.header.seq = num_diffs - 1;

    // Add the diff to the map diffs array
    mapdiffs.octomaps.push_back(msg);
    mapdiffs.num_octomaps = num_diffs;
    pub_mapdiffs.publish(mapdiffs);

    // Publish the number of diffs so multi_agent doesn't have to subscribe to the whole map
    std_msgs::UInt32 size_msg;
    size_msg.data = num_diffs;
    pub_size.publish(size_msg);
  }

  // Remove all of the nodes whether we used them or not, for the next iter
  tree_diff->clear();

  // Merge each neighbors' diff map to the merged map
  bool overwrite_node;
  for (int i=0; i < neighbors.num_neighbors; i++) {
    std::string nid = neighbors.neighbors[i].owner;
    // Check each diff for new ones to merge
    for (int j=0; j < neighbors.neighbors[i].num_octomaps; j++) {
      uint32_t cur_seq = neighbors.neighbors[i].octomaps[j].header.seq;
      bool exists = std::count(seqs[nid.data()].cbegin(), seqs[nid.data()].cend(), cur_seq);

      if (!exists) {
        // ROS_INFO("%s Merging neighbor %s seq %d", id.data(), nid.data(), cur_seq);
        seqs[nid.data()].push_back(cur_seq);
        if (octo_type == 0)
          tree_temp = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(neighbors.neighbors[i].octomaps[j]);
        else
          tree_temp = (octomap::OcTree*)octomap_msgs::fullMsgToMap(neighbors.neighbors[i].octomaps[j]);

        // TODO Still problem where only replacing, not merging.  If multiple neighbors see the same node, only the last one received gets used
        // If it's latest, merge and append.  If not, only append
        if (cur_seq >= *std::max_element(seqs[nid.data()].cbegin(), seqs[nid.data()].cend()))
          overwrite_node = true;
        else
          overwrite_node = false;

        // Merge neighbor map
        merge_maps(tree_merged, tree_temp, full_merge, false, overwrite_node, free_prioritize);

        if (publish_diff_pcl)
          merge_maps(tree_pcl, tree_temp, true, false, false, free_prioritize);

        // Free the memory before the next neighbor
        delete tree_temp;
      }
    }
  }

  // Publish the merged PCL
  if (publish_merged_pcl && (tree_merged->getRoot() != NULL)) {
    pub_merged_pcl.publish(buildPCL(tree_merged));
  }

  // Publish the PCL diff
  if (publish_diff_pcl && (tree_pcl->getRoot() != NULL)) {
    pub_diff_pcl.publish(buildPCL(tree_pcl));
    tree_pcl->clear();
  }

  // Prune and publish the Octomap
  tree_merged->prune();
  if (octo_type == 0)
    octomap_msgs::binaryMapToMsg(*tree_merged, msg);
  else
    octomap_msgs::fullMapToMsg(*tree_merged, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.id = "OcTree"; // Required to convert OcTreeStamped into regular OcTree
  pub_merged.publish(msg);

  if (type == "robot") delete tree_sys;
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "octomap_merger", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  double rate;
  std::string nn = ros::this_node::getName();
  nh.param(nn + "/rate", rate, (double)0.1);

  OctomapMerger *octomap_merger = new OctomapMerger(&nh);

  ros::Rate r(rate);
  while(nh.ok()) {
    ros::spinOnce();
    if(octomap_merger->myMapNew || octomap_merger->otherMapsNew) {
      octomap_merger->myMapNew = false;
      octomap_merger->otherMapsNew = false;
      octomap_merger->merge();
    }
    r.sleep();
  }
  return 0;
}
