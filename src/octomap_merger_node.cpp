#include <octomap_merger.h>

OctomapMerger::OctomapMerger(ros::NodeHandle* nodehandle):nh_(*nodehandle) {
    ROS_INFO("Constructing OctomapMerger Class");

    std::string nn = ros::this_node::getName();
    // Load parameters from launch file
    nh_.param<std::string>(nn + "/vehicle", id, "H01");
    // Type of agent (robot or base)
    nh_.param<std::string>(nn + "/type", type, "robot");
    // Set the merger type: 0: Octomap, 1: PCL-Kyle, 2: PCL-Jessup
    nh_.param(nn + "/merger", merger, 0);
    // Full merging or prioritize own map
    nh_.param(nn + "/full_merge", full_merge, true);
    // Keep free space from any agent instead of adding obstacles
    nh_.param(nn + "/free_prioritize", free_prioritize, false);
    // Octomap type: 0: Binary, 1: Full
    nh_.param(nn + "/octoType", octo_type, 0);
    // Map resolution
    nh_.param(nn + "/resolution", resolution, (double)0.2);
    // Map size threshold to trigger a map merge
    nh_.param(nn + "/mapThresh", map_thresh, 500);

    // Topics for Subscribing and Publishing
    nh_.param<std::string>(nn + "/mapTopic", map_topic, "octomap_binary");
    nh_.param<std::string>(nn + "/neighborsTopic", neighbors_topic, "neighbor_maps");
    nh_.param<std::string>(nn + "/mergedTopic", merged_topic, "merged_map");
    nh_.param<std::string>(nn + "/mergedSizeTopic", merged_size_topic, "merged_size");
    nh_.param<std::string>(nn + "/pclTopic", pcl_topic, "pc2_out");

    initializeSubscribers();
    initializePublishers();
    myMapNew = false;
    otherMapsNew = false;

    // Initialize Octomap holders once, assign/overwrite each loop
    treep = new octomap::OcTree(resolution);
    treem = new octomap::OcTree(resolution);
    tree1 = new octomap::OcTree(resolution);
    tree2 = new octomap::OcTree(resolution);
    treep_size = 0;
    if (type == "robot") tree1_last_size = 0;
    else tree1_last_size = map_thresh;
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
    pub_size = nh_.advertise<std_msgs::Float64>(merged_size_topic, 1, true);
    if (type == "base")
        pub_pcl = nh_.advertise<sensor_msgs::PointCloud2>(pcl_topic, 1, true);
}

// Callbacks
void OctomapMerger::callback_myMap(const octomap_msgs::OctomapConstPtr& msg) {
  // ROS_INFO("my_map callback");
  myMap = *msg;
  myMapNew = true;
}

void OctomapMerger::callback_neighborMaps(
                const octomap_merger::OctomapArrayConstPtr& msg) {
  // ROS_INFO("neighbors callback");
  neighbors = *msg;
  otherMapsNew = true;
}

void OctomapMerger::octomap_to_pcl(octomap::OcTree* myTree,
                    sensor_msgs::PointCloud2Ptr occupiedCellsMsg) {
  ROS_INFO("Octomap_to_pcl");
  double size, value;
  float lower_corner[3];
  int idk, depth, width;
  int lowest_depth = (int)myTree->getTreeDepth();
  int count = 0;
  int freeCount = 0;
  int occCount = 0;
  float voxel_size = (float)myTree->getResolution();

  PointT point;
  PointCloud::Ptr occupiedCells(new PointCloud);
  ROS_INFO("Iterating through tree");
  for (octomap::OcTree::leaf_iterator it = myTree->begin_leafs(),
                        end = myTree->end_leafs(); it != end; ++it) {
    depth = (int)it.getDepth();
    point.x = (float)it.getX();
    point.y = (float)it.getY();
    point.z = (float)it.getZ();
    size = it.getSize();

    if (myTree->isNodeOccupied(*it)) {
      if(depth == lowest_depth) {
        occupiedCells->points.push_back(point);
      } else {
        width = (int)std::pow(2.0, (double)(lowest_depth-depth));
        lower_corner[0] = point.x - size/2.0 + voxel_size/2.0;
        lower_corner[1] = point.y - size/2.0 + voxel_size/2.0;
        lower_corner[2] = point.z - size/2.0 + voxel_size/2.0;
        for (int i=0; i < width; i++) {
          point.x = lower_corner[0] + i*voxel_size;
          for (int j=0; j < width; j++) {
            point.y = lower_corner[1] + j*voxel_size;
            for (int k=0; k < width; k++) {
              point.z = lower_corner[2] + k*voxel_size;
              if (value) {
                occupiedCells->points.push_back(point);
              }
            }
          }
        }
      }
    }
  }

  pcl::toROSMsg(*occupiedCells, *occupiedCellsMsg);
}

void OctomapMerger::merge() {
  if (octo_type == 0)
    tree1 = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(myMap);
  else
    tree1 = (octomap::OcTree*)octomap_msgs::fullMsgToMap(myMap);

  if (!tree1 && (type == "robot")) return;

  // Create pointers if we're using PCL conversion
  sensor_msgs::PointCloud2Ptr myMapMsg(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2Ptr mergedMapMsg(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2Ptr neighborMapMsg(new sensor_msgs::PointCloud2);

  // Check the number of leaves have changed enough to merge self
  // Get rid of this with a bounding box on the tree
  size_t tree1_size;
  double tree2_size, tree2_last_size;
  std::string nid;

  // Merge all of the neighbors into a tree first
  // For each map in the neighbor set
  for (int i=0; i < neighbors.num_octomaps; i++) {
    if (octo_type == 0)
      tree2 = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(neighbors.octomaps[i]);
    else
      tree2 = (octomap::OcTree*)octomap_msgs::fullMsgToMap(neighbors.octomaps[i]);

    // Check the size of the map has changed enough to merge this neighbor
    // Also check that the neighbor map is bigger than our merged map
    nid = neighbors.owners[i];
    tree2_last_size = treen_last_size[nid.data()];
    tree2_size = neighbors.sizes[i];
    if ((tree2_size < tree2_last_size - map_thresh) ||
       ((tree2_size > tree2_last_size + map_thresh) &&
        (tree2_size > treep_size))) {
      ROS_INFO("%s Merging neighbor %s", id.data(), nid.data());

      // Merge neighbor map
      if (merger > 0) {
        // Multiple PCL conversion methods
        if (merger == 1) {
          octomap_to_pcl(tree2, neighborMapMsg);
        } else {
          PointCloud::Ptr occupiedCells(new PointCloud);
          tree2PointCloud(tree2, *occupiedCells);
          pcl::toROSMsg(*occupiedCells, *neighborMapMsg);
        }
        // Add map to merge
        pcl::concatenatePointCloud(*mergedMapMsg, *neighborMapMsg, *mergedMapMsg);
      } else {
        // Octomap merging
        treen_last_size[nid.data()] = merge_maps(treem, tree2, true, free_prioritize);
      }
    }

    // Free the memory before the next neighbor
    delete tree2;
  }

  // Merge into the published tree
  if (full_merge) {
    if (type == "robot") tree1_size = tree1->getNumLeafNodes();
    else tree1_size = map_thresh;

    if ((tree1_size < tree1_last_size - map_thresh) ||
        (tree1_size > tree1_last_size + map_thresh)) {
      tree1_last_size = tree1_size;

      // Get the current self/merged map
      if (merger > 0) {
        // Multiple PCL conversion methods
        if (merger == 1) {
          // Could store the merged PCL in memory, or we just do the conversion each time
          octomap_to_pcl(treem, mergedMapMsg);
          octomap_to_pcl(tree1, myMapMsg);
        } else {
          // This doesn't currently work properly with the stored merged map!
          PointCloud::Ptr occupiedCells(new PointCloud);
          tree2PointCloud(tree1, *occupiedCells);
          pcl::toROSMsg(*occupiedCells, *myMapMsg);
        }
        // Add map to merge
        pcl::concatenatePointCloud(*mergedMapMsg, *myMapMsg, *mergedMapMsg);
      } else {
        // Octomap merging between self and the saved merged map
        // ROS_INFO("Merging self...");
        tree1_size = merge_maps(treem, tree1, true, free_prioritize);
      }
    }

    // The published map is the fully merged map
    treep = treem;
  } else {
    // The published map starts with the self map and only appends merged
    treep = tree1;
    merge_maps(treep, treem, false, free_prioritize);
  }

  // Convert PCL to Octomap
  if (merger > 0) {
    // Clear the tree so we can rebuild it
    treep->clear();
    octomap::Pointcloud octoPCL;
    octomap::point3d octo_points;

    // Convert the ROS PCL to a regular PCL
    pcl::PointCloud<pcl::PointXYZ> tmpPCL;
    pcl::fromROSMsg(*mergedMapMsg, tmpPCL);

    // Add all PointCloud2 points to the Octomap Pointcloud
    for (int i=0; i < tmpPCL.points.size(); i++) {
      octo_points(0) = tmpPCL.points[i].x;
      octo_points(1) = tmpPCL.points[i].y;
      octo_points(2) = tmpPCL.points[i].z;
      octoPCL.push_back(octo_points);
    }

    // Generate Octomap from Pointcloud
    treep->insertPointCloud(octoPCL, point3d(0,0,0));
  }

  // Get the size of the map and publish it so multi-agent can send it
  treep_size = 0;
  treep->expand();
  for (OcTree::leaf_iterator it = treep->begin_leafs(); it != treep->end_leafs(); ++it) {
    treep_size += it.getSize();
  }
  std_msgs::Float64 size_msg;
  size_msg.data = treep_size;
  pub_size.publish(size_msg);

  // For Base Station, convert to PCL before pruning and publish
  if (type == "base") {
    sensor_msgs::PointCloud2 pcl;
    PointCloud::Ptr occupiedCells(new PointCloud);
    tree2PointCloud(treep, *occupiedCells);
    pcl::toROSMsg(*occupiedCells, pcl);
    pcl.header.stamp = ros::Time::now();
    pcl.header.frame_id = "world";
    pub_pcl.publish(pcl);
  }

  // Prune and publish the Octomap
  treep->prune();
  octomap_msgs::Octomap msg;
  if (octo_type == 0)
    octomap_msgs::binaryMapToMsg(*treep, msg);
  else
    octomap_msgs::fullMapToMsg(*treep, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  pub_merged.publish(msg);
  delete tree1;
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "octomap_merger", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  double rate;
  std::string nn = ros::this_node::getName();
  nh.param(nn + "/rate", rate, (double)1.0);

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
