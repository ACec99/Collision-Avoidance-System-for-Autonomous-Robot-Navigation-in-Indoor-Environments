#include "eigen_01_point_loading.h"
#include "eigen_02_isometry.h"
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv) {
  if (argc<2)
    return -1;
  // we open a file stream;

  Isometry3f iso;
  iso.rotation() <<
    0, -1, 0,
    1,  0, 0,
    0,  0, 1;
  iso.translation() << 0, 0, 0;
  
  std::ifstream is(argv[1]);
  Vector3fVector points;
  int num_points = loadPoints(points, is);
  cerr << "I read" << num_points << "from the stream " << endl;
  savePoints(std::cerr, points);


  // we apply the transform to all points
  for (auto& v: points) {
    v=iso*v;
  }
  cerr << "transform" << endl;
  savePoints(std::cout, points);


  /* TODO for you 

     1. change the above program to operate on 2D isometries
     2. modify the program to write the points transformed in another vector
     3. compute an inverted isometry (use inverse()) an apply this transform to the
        vector of step 2;
     4. verify that the input and vector at step 3 are the same

     5. write a different application that takes three command line arguments:
        - a file with a 2D transform (x,y, theta)
        - a file with the input points
        - a file where to write (use ofstream(<filename>)
        applies the transform to each input point and saves the output to transformed point

        hint: recover the transform from x,y,theta as
        R(0,0)=cos(theta); R(0,1)=-sin(theta);
        R(1,0)=sin(theta); R(1,1)=cos(theta);
        t(0)=x;
        t(1)=y;
  */

}
