#include "eigen_01_point_loading.h"
#include "eigen_04_kdtree.h"
#include <iostream>
#include <fstream>

using namespace std;

using ContainerType = std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >;

int main(int argc, char** argv) {
  // generate 1000 random points
  ContainerType kd_points(1000);
  for (auto& v: kd_points) {
    v=Vector3f::Random()*100;
  }

  // construct a kd_tree, with leaf size 10
  // the construction reshuffles the items in the container
  auto kd_tree=TreeNodeBase_<ContainerType>::buildTree(kd_points.begin(),
                                                        kd_points.end(),
                                                        10);
  
  
  // we search each point in the input set. We need to find a match
  for (auto p: kd_points) {
    ContainerType neighbors;
    kd_tree->search(neighbors, p, 0.1);
    cerr << neighbors.size() << " ";
  }
  cerr << endl;


  /* 
     todo for you:
     1. modify this program to operate on 2D points stored in std::lists;
     2. modify this program so that it loads two sets of 2D points, and looks fot the
        best match of each point in the second set and in the first set,
        within a user specified range
        the program should output a text file in the form

        pa1.x pa1.y
        pb1.x pb1.y
        <emptyline>
        pa2.x pa2.y
        pb2.x pb2.y
        <emptyline>
        ...
        
        if no match is found no pair is written.
        Here pa and mb are the points in the first and second set
   */

  
}



