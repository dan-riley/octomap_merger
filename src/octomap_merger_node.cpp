#include <octomap_merger.h>

OctomapMerger::OctomapMerger(ros::NodeHandle* nodehandle, int merger_i,
                             double resolution):nh_(*nodehandle) {
    ROS_INFO("Constructing OctomapMerger Class");
    initializeSubscribers();
    initializePublishers();
    myMapNew = false;
    otherMapsNew = false;
    merger = merger_i;
    res = resolution;

    // Initialize Octomap holders once, assign/overwrite each loop
    base = new typename OcTreeBase<OcTreeNode>::OcTreeBase(resolution);
    tree1 = new octomap::OcTree(resolution);
    tree2 = new octomap::OcTree(resolution);
}

// Destructor
OctomapMerger::~OctomapMerger() {
}

void OctomapMerger::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    // Make this a launch file parameter
    sub_mymap = nh_.subscribe("octomap_binary", 100,
                              &OctomapMerger::callback_myMap, this);
    sub_neighbors = nh_.subscribe("neighbor_maps", 100,
                                  &OctomapMerger::callback_neighborMaps, this);
}

void OctomapMerger::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    // Make this a launch file parameter
    pub_merged = nh_.advertise<octomap_msgs::Octomap>("merged_map", 10, true);
}

// Callbacks
void OctomapMerger::callback_myMap(const octomap_msgs::OctomapConstPtr& msg) {
  ROS_INFO("my_map callback");
  myMap = *msg;
  myMapNew = true;
}

void OctomapMerger::callback_neighborMaps(
                const octomap_merger::OctomapArrayConstPtr& msg) {
  ROS_INFO("neighbors callback");
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
  ROS_INFO("Entered Merge Function");
  // Change to allow binary or full, from launch file
  tree1 = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(myMap);

  // Create pointers if we're using PCL conversion
  ROS_INFO("Creating pointers");
  sensor_msgs::PointCloud2Ptr myMapMsg(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2Ptr mergedMapMsg(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2Ptr neighborMapMsg(new sensor_msgs::PointCloud2);

  // PCL merging
  if (merger > 0) {
    ROS_INFO("Converting my map");
    // Multiple PCL conversion methods
    if (merger == 1) {
      octomap_to_pcl(tree1, myMapMsg);
    } else {
      PointCloud::Ptr occupiedCells(new PointCloud);
      tree2PointCloud(tree1, *occupiedCells);
      pcl::toROSMsg(*occupiedCells, *myMapMsg);
    }
    // Add map to merge
    ROS_INFO("Adding Map to Merge");
    pcl::concatenatePointCloud(*mergedMapMsg, *myMapMsg, *mergedMapMsg);
  }

  // For each map in the neighbor set
  for (int i=0; i < neighbors.num_octomaps; i++) {
    ROS_INFO("Merging neighbor...");
    tree2 = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(neighbors.octomaps[i]);

    // PCL merging
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
      ROS_INFO("Adding NeighborMap to Merge");
      pcl::concatenatePointCloud(*mergedMapMsg, *neighborMapMsg, *mergedMapMsg);
    } else {
      // Octomap merging
      merge_maps(base, tree1, tree2);
    }
    delete tree2;
  }

  // Convert PCL to Octomap
  if (merger > 0) {
    // Clear the tree so we can rebuild it
    tree1->clear();
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
    tree1->insertPointCloud(octoPCL, point3d(0,0,0));
  }

  // Prune and publish the Octomap
  tree1->prune();
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(*tree1, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  pub_merged.publish(msg);
  delete tree1;
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "octomap_merger");
  ros::NodeHandle nh;

  int rate;
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate, int(1));

  // Map resolution
  double resolution;
  nh.param("octomap_merger/resolution", resolution, (double)0.2);

  // Set the merger type: 0: Octomap, 1: PCL-Kyle, 2: PCL-Jessup
  int merger;
  nh.param("octomap_merger/merger", merger, 0);

  OctomapMerger *octomap_merger = new OctomapMerger(&nh, merger, resolution);

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
