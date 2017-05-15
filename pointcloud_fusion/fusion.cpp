#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

// This is the main function
int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB> pos1, pos2, pos1_t;


  pcl::io::loadPCDFile ("../data/pcd_pos1/pcd_2.pcd", pos1);
  pcl::io::loadPCDFile ("../data/pcd_pos2/pcd_2.pcd", pos2);

  /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */

  Eigen::Matrix4d transform1 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d transform2 = Eigen::Matrix4d::Identity();

  std::ifstream infile("../data/rts.txt");

  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
    {
      infile >> transform1(i,j);
    }
  }
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
    {
      infile >> transform2(i,j);
    }
  }
  infile.close();
  /*transform1 << 0.997577 , 0.0346118,  0.0603459,   0.805226,
                -0.0319782,   0.998516, -0.0440743,  -0.215247,
                -0.0617819,  0.0420377,  0.997204,  -0.111191,
                  0,          0,          0,         1;


  transform2 <<    0.998948,   0.0400165,   0.0223957,   -0.525663,
                  -0.0399718,    0.999198, -0.00244055,    -0.36611,
                  -0.0224754,  0.00154279,    0.999746,  -0.0679732,
                  0,           0,           0 ,          1;*/

  std::cout << transform1 << "\n" << transform2 <<"\n";

  Eigen::Matrix4d transform = transform2*(transform1.inverse());

  std::cout << "Final transform from camera1 to camera2 is:\n" << transform <<"\n";


  

  pcl::transformPointCloud (pos1, pos1_t, transform);

  pos2 += pos1_t;
  pcl::io::savePCDFileASCII ("../data/final_cloud.pcd", pos2);




  return 0;
}
