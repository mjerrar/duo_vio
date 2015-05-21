/*
 * EKFOdometry.h
 *
 *  Created on: Oct 16, 2014
 *      Author: tpetri
 *
 *  Thread-safe KLT grid based tracker
 */

#ifndef EKFODOMETRY_H_
#define EKFODOMETRY_H_

#include "threading/Mutex.h"
#include "threading/Thread.h"

#include <opencv2/core/core.hpp>
#include <list>
#include <vector>

// #define MATLAB TODO uncomment if neccecary

#define PATCHSIZE 7
#define LEVELS 4

#ifndef MATLAB
#include "EKFPhotometric.h"
#endif

bool vision_process_frame(const cv::Mat &img);
bool vision_draw_tracks(cv::Mat &output);

class EKFOdometry : public ait::Thread
{
public:
  EKFOdometry(double f, double cx, double cy, const std::vector<float> kc, int patchSize = 9, int gridSizeX = 4, int gridSizeY = 4, int targetNumPoints = 8, double minScore = 50, int minPixDist = 30)
: f(f), cx(cx), cy(cy)
, kc(kc)
, patchSize(patchSize)
, halfPatchSize((patchSize+1)/2)
, gridSizeX(gridSizeX)
, gridSizeY(gridSizeY)
, targetNumPoints(targetNumPoints)
, minScore(minScore)
, minDistSqr(minPixDist*minPixDist)
, imgCond(imgMutex)
, nextImageWaiting(false)
#ifndef MATLAB
,  m_ekf(NULL)
#endif
{ cornerStatus = (unsigned char *) malloc(targetNumPoints * sizeof(unsigned char)); memset(cornerStatus, 0, targetNumPoints * sizeof(unsigned char)); gridScore = (double *) malloc(targetNumPoints * sizeof(double)); gridPoint = new cv::Point2f[targetNumPoints]; gridOccupied = (unsigned char *) malloc(targetNumPoints * sizeof(unsigned char)); points = (double *)malloc(2*targetNumPoints*sizeof(double)); preCorners.resize(targetNumPoints); /* for (int l = 0; l < 4; l++) img_pyr[l].img = NULL;*/}

  ~EKFOdometry() { free(cornerStatus); free(gridScore); delete [] gridPoint; free(gridOccupied); free(points); }

public:
  //initialize filter data structures
  void InitFilter(const double *initialPose = NULL);
  //queue an image to be process by the update
  bool ProcessFrame(const cv::Mat &img);
  //queue IMU data to be processed by the prediction
  bool ProcessIMUData(double dt, float a[3], float w[3]);
  //draw an OpenCV window showing the currently tracked points
  bool DrawTracks(cv::Mat &output, bool undist = false);

  //return the current position
  void GetPosition(double &x, double &y, double &z) const;

  //this is the thread function where the pred-upd iterations
  //are running that process the IMU and image data that
  //here put into the filter by the ProcessX functions
  void Run();

#ifndef MATLAB
  sfm::EKFPhotometric<PATCHSIZE, LEVELS> *GetEKF() { return m_ekf; }
#endif


protected:
  void predict();
  void update();

  void initMorePoints(const cv::Mat &img, size_t level);
  double shiTomasiScore(const cv::Mat &image, const cv::Point &position, int halfWindow) const;

protected:
  struct GridOrder
  {
    GridOrder() : maxScore(0), gx(0), gy(0) {}

    struct PointWithScore
    {
      PointWithScore(double x, double y) : x(x), y(y), score(0) {}
      double x, y, score;
    };

    double maxScore;
    std::vector<PointWithScore> points;
    unsigned char gx, gy;
  };

  struct IMUData
  {
    double dt;
    double a[3];
    double w[3];
    double m[3];
  };

protected:
  unsigned char *cornerStatus;
  double *gridScore;
  cv::Point2f *gridPoint;
  unsigned char *gridOccupied;

  cv::Mat pyramid[LEVELS];
  cv::Mat pyramid_col[LEVELS];
  cv::Mat prev_img;

  std::vector<cv::Point2f> preCorners;
  double *points;

  double f, cx, cy;
  std::vector<float> kc;

  const int patchSize;
  const int halfPatchSize;
  const int gridSizeX;
  const int gridSizeY;
  const int targetNumPoints;
  const double minScore;
  const double minDistSqr;

  //synchronization for img buffer
  //thats always the most recent image
  ait::Mutex imgMutex;
  ait::Condition imgCond;
  bool nextImageWaiting;
  cv::Mat nextImg;

  //synchronization for imu buffer
  //thats a list of all given measurements
  //the list will be emptied by the
  //prediction function
  ait::Mutex imuMutex;
  std::list<IMUData *> imuBuffer;

#ifndef MATLAB
  sfm::EKFPhotometric<PATCHSIZE, LEVELS> *m_ekf;
#endif

//  struct0_T img_pyr[4];
};

#endif /* EKFODOMETRY_H_ */
