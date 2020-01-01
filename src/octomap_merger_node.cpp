#include <octomap_merger.h>

OctomapMerger::OctomapMerger(ros::NodeHandle* nodehandle):nh_(*nodehandle) {
    ROS_INFO("Constructing OctomapMerger Class");
    initializeSubscribers();
    initializePublishers();
    myMapNew = false;
    otherMapsNew = false;

    // Initialize Octomap holders once, assign/overwrite each loop
    // Need to add resolution to the launch file
    base = new typename OcTreeBase<OcTreeNode>::OcTreeBase(0.2);
    tree1 = new octomap::OcTree(0.2);
    tree2 = new octomap::OcTree(0.2);
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

void OctomapMerger::merge() {
  ROS_INFO("Entered Merge Function");
  // Change to allow binary or full, from launch file
  tree1 = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(myMap);

  // For each map in the neighbor set
  for (int i=0; i < neighbors.num_octomaps; i++) {
    tree2 = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(neighbors.octomaps[i]);
    merge_maps(base, tree1, tree2);
  }

  // Prune and publish the Octomap
  tree1->prune();
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(*tree1, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  pub_merged.publish(msg);
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "octomap_merger");
  ros::NodeHandle nh;

  int rate;
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate, int(1));

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
