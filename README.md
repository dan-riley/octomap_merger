# octomap_merger

Merges multiple Octomaps and optionally aligns them using ICP prior to merging.

Derived from James Jessup, Sidney N. Givigi, and Alain Beaulieu. "Merging of octree based 3d occupancy grid maps." In 2014 IEEE International Systems Conference Proceedings, pp. 371-377. IEEE, 2014. DOI: 10.1109/SMC.2014.6974556

<pre>
Usage: octomap_merger &lt;input_file_1> &lt;input_file_2> &lt;output_file\> [align]
                      [&lt;translation_x\> &lt;translation_y\> &lt;translation_z\>]
                      [&lt;roll\> &lt;pitch\> &lt;yaw\>]
</pre>

Input files can be binary or full, but both must the same.
Output can be either, but does not have to be the same as input.
Use .bt or .ot file extension to specify

Add 'align' option to run ICP alignment on maps before merging.

## Source Description

octomap_merger.cpp - Command-line merger

octomap_merger_node.cpp - ROS node for merging multiple maps in an array

map_merger.cpp - Core functions that manage actual Octomap merging

icp_align.cpp - Converts Octomaps to point clouds, finds ICP alignment, and transforms the second map to align with the first.
