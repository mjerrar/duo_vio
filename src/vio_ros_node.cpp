#include <ros/ros.h>

#include <fstream>
using namespace std;

#include "localization.h"

cv::Size m_size;						///< Image Size for this calibration
std::vector<float> m_focal; ///< The focal length
std::vector<float> m_cc; 		///< The principal point location
std::vector<float> m_kc;		///< Distortion coefficients

bool read_calibration_file(const char *filename);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vio_ros_node");
  
  if (!read_calibration_file("/home/michael/Projects/AIT/hcf_onboard/src/VIO_ROS/src/camera.cal"))
  {
    printf("Exiting because no calibration for camera.\n");
    exit(-1);
  }

  Localization estimation(m_focal, m_cc, m_kc);

  ros::spin();
  return 0;
}

bool read_calibration_file(const char *filename)
{
	// Load the calibration file
    ifstream calfile;
    calfile.open(filename);
    if (!calfile) {
        cout << "Unable to open file " << filename << endl;
        return false;
    }

    // Ignore the fileheader for now
    string fileHeader;
    getline(calfile,fileHeader);

    m_cc.resize(2);
    m_focal.resize(2);
    m_kc.resize(5);

    // Process the values
    calfile >> m_size.width; calfile >> m_size.height; // get image size
    calfile >> m_cc[0]; calfile >> m_cc[1]; // get camera center
    calfile >> m_focal[0]; calfile >> m_focal[1]; // get focal length
    calfile >> m_kc[0]; calfile >> m_kc[1]; calfile >> m_kc[2]; calfile >> m_kc[3]; calfile >> m_kc[4]; // get distortion parameters

    // Close the calibration file
    calfile.close();
	return true;
}
