#include <octomap_merger.h>

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

    align_maps(tree1, tree2, translation, roll, pitch, yaw, res);
  }

  merge_maps(base, tree1, tree2);

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
