#include "EKFOdometry.h"

#define USE_KLT

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#ifdef USE_KLT
#include <opencv2/video/tracking.hpp>
#endif

#include <algorithm>
#include <list>
#include <stdio.h>

//codegen files
// #include "../codegen_SLAMpred/SLAM_pred.h"
// #include "../codegen_SLAMupd/SLAM_updIT.h"

//C EKF files
#include "camera_models/CameraPinhole.h"

double qv = 2;
double qw = 4;
double imNoise = 1;

static const int numStates = 13;
static const int numPointsPerAnchor = 1;
static const int numAnchors = 16;
static const int fullAnchorParam = 1;

static const int numStatesPerAnchor = 7 + (int)numPointsPerAnchor;
static const int numStatesTotal = numStates + numAnchors*numStatesPerAnchor;

#ifdef MATLAB
static double P_apr[numStatesTotal*numStatesTotal];
static double P_apo[numStatesTotal*numStatesTotal];
static double x_apr[numStatesTotal];
#endif
static double x_apo[numStatesTotal];

static bool verbose = true;

//outputs for debug
static double h_u_apo[32];
static double h_u_apr[32];

#ifndef _MSC_VER
#include <unistd.h>
#include <sys/time.h>
static uint64_t get_now() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((uint64_t) t.tv_sec) * 1000000 + (uint64_t) t.tv_usec;
}
#else
#include <windows.h>
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
static uint64_t get_now() {
    FILETIME ft;
    long long tmpres = 0;
    static int tzflag;

  //Contains a 64-bit value representing the number of 100-nanosecond intervals since January 1, 1601 (UTC).
  GetSystemTimeAsFileTime(&ft);

  tmpres |= ft.dwHighDateTime;
  tmpres <<= 32;
  tmpres |= ft.dwLowDateTime;

  //tempres is in 100ns increments
  //Convert it to us
  tmpres /= 10;

  /*converting file time to unix epoch*/
  tmpres -= DELTA_EPOCH_IN_MICROSECS;
    return tmpres;
}
#endif

void EKFOdometry::InitFilter(const double *initialPose)
{
#ifdef MATLAB
  memset(x_apr, 0, sizeof(double)*numStatesTotal);
  memset(x_apo, 0, sizeof(double)*numStatesTotal);

  double initialPoseDefault[7] = { 0, 0, 0, 1, 0, 0, 0 };

  //set orientation
  if (!initialPose)
  {
    initialPose = initialPoseDefault;
  }

  for (int i = 0; i < 7; i++)
  {
    x_apr[i] = initialPose[i];
    x_apo[i] = initialPose[i];
    for (int a = 0; a < numAnchors; a++)
    {
      x_apr[numStates+a*numStatesPerAnchor+i] = initialPose[i];
      x_apo[numStates+a*numStatesPerAnchor+i] = initialPose[i];
    }
  }

  //set covariance to ones
  double init = 1.;
  for (int i = 0; i < numStatesTotal; i++)
  {
    for (int ii = 0; ii < numStatesTotal; ii++)
    {
      P_apr[i*numStatesTotal+ii] = init;
      P_apo[i*numStatesTotal+ii] = init;
    }
  }

  memset(cornerStatus, 0, targetNumPoints);

  SLAM_updIT_initialize();
#else
  sfm::CameraPinhole<double>::Parameters params(std::string("cam1"), 640, 480, Eigen::Matrix<double, 2, 1>(f, f), Eigen::Matrix<double, 2, 1>(cx, cy), std::vector<double>());
  sfm::CameraPinhole<double> *cam = new sfm::CameraPinhole<double>(params);
  m_ekf = new sfm::EKFPhotometric<PATCHSIZE, LEVELS>(*cam);
  m_ekf->init(Eigen::Isometry3d::Identity());
#endif
}

bool EKFOdometry::ProcessFrame(const cv::Mat &img)
{
  //take the image and copy it to the local buffer
  //always overwrite the buffer since we are only interested in the most current image

  imgMutex.Lock();
  img.copyTo(nextImg);
  //notify the thread if it is waiting for new images
  nextImageWaiting = true;
  imgCond.NotifyAll();
  imgMutex.Unlock();

  return false;
}

bool EKFOdometry::ProcessIMUData(double dt, float a[3], float g[3])
{
  IMUData *newData = new IMUData;

  for (int i = 0; i < 3; i++)
  {
    newData->dt = dt;
    newData->a[i] = 0.;//a[i];
    newData->w[i] = 0.;//g[i];
    newData->m[i] = 0.;
  }

  //add data to the buffer
  imuMutex.Lock();
  imuBuffer.push_back(newData);
  imuMutex.Unlock();

  return true;
}

bool EKFOdometry::DrawTracks(cv::Mat &output, bool undist)
{
  imgMutex.Lock();
  if (prev_img.empty()) { imgMutex.Unlock(); return false; }

  if (undist)
  {
    cv::Mat undist;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << f, 0, cx, 0, f, cy, 0, 0, 1);
    cv::undistort(prev_img, undist, cameraMatrix, kc);
    cv::cvtColor(undist, output, CV_GRAY2BGR);
  }
  else
    cv::cvtColor(prev_img, output, CV_GRAY2BGR);

  std::vector<cv::Point2f> tracks;
  for (int i = 0; i < targetNumPoints; i++)
  {
    tracks.push_back(cv::Point2f(points[2*i+0], points[2*i+1]));
  }
  imgMutex.Unlock();

  for (int i = 0; i < targetNumPoints; i++)
  {
    if (cornerStatus[i] > 0)
    {
      cv::circle(output, tracks[i], 3, CV_RGB(0, 250, 0), CV_FILLED);
      cv::circle(output, cv::Point2f(h_u_apo[2*i+0], h_u_apo[2*i+1]), 3, CV_RGB(250, 250, 0), CV_FILLED);
    }
  }
  return true;
}

void EKFOdometry::GetPosition(double &x, double &y, double &z) const
{
#ifdef MATLAB
  x = x_apo[0];
  y = x_apo[1];
  z = x_apo[2];
#endif
}

void EKFOdometry::Run()
{
  while(!mbStopping)
  {
    uint64_t tic1 = get_now();
    predict();
    uint64_t toc1 = get_now();
    if (verbose)
      printf("predict: %6.1f ms\n", (double)(toc1 - tic1) / 1000);

    uint64_t tic2 = get_now();
    update();
    uint64_t toc2 = get_now();
    if (verbose)
      printf("update: %6.1f ms\n", (double)(toc2 - tic2) / 1000);

    //test for filter fail, in that case reset filter and start at current position
    if (!(sqrt(x_apo[0]*x_apo[0]+x_apo[1]*x_apo[1]+x_apo[2]*x_apo[2]) < 3.))
    {
      prev_img.release();
      InitFilter();
    }
  }
}

void EKFOdometry::predict()
{
  double updateVectIMU[3] = {};
  double R_ci[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1};

  //first lock the buffer and copy all pointers to a local buffer and unlock the public buffer for new data
  std::list<IMUData *> localBuffer;
  imuMutex.Lock();

//  //this test should be done with conditions, but it should anyway only happen at program start once or twice...
//  while (imuBuffer.size() == 0) { imuMutex.Unlock(); usleep(5000); imuMutex.Lock(); }

  for (std::list<IMUData *>::iterator it = imuBuffer.begin(); it != imuBuffer.end(); ++it)
  {
    localBuffer.push_back(*it);
  }
  imuBuffer.clear();
  imuMutex.Unlock();

  if (localBuffer.size() != 1) printf("PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM!!!!!!!!!!!!!!!!!! %zu\n", localBuffer.size());

  //now process the local buffer
  for (std::list<IMUData *>::iterator it = localBuffer.begin(); it != localBuffer.end(); ++it)
  {
    IMUData *data = *it;

//    printf("PREDICT: %f %f %f %f %f %f %f %f %f %f\n", data->dt, data->w[0], data->w[1], data->w[2], data->a[0], data->a[1], data->a[2], data->m[0], data->m[1], data->m[2]);
#ifdef MATLAB
    //Predict with IMU data
    SLAM_pred(P_apo, x_apo, data->dt, qv, qw, 0., data->w, data->a, data->m, updateVectIMU, numPointsPerAnchor, numAnchors, fullAnchorParam, R_ci, x_apr, P_apr);

    //copy apriori to aposteriori for the case there are more IMU measurements before the next update
    memcpy(x_apo, x_apr, numStatesTotal*sizeof(double));
    memcpy(P_apo, P_apr, numStatesTotal*numStatesTotal*sizeof(double));
#else
    m_ekf->predict(data->dt);
#endif
  }

//  double fake[3] = {};
//  SLAM_pred(P_apo, x_apo, 0.033, qv, qw, 0., fake, fake, fake, updateVectIMU, numPointsPerAnchor, numAnchors, fullAnchorParam, R_ci, x_apr, P_apr);
}

void EKFOdometry::update()
{
//  printf("UPDATE\n");
  cv::Mat img;

  //wait for next img
  //typically there is no wait time
  imgMutex.Lock();
  imgCond.Wait();
  if (nextImageWaiting)
  {
    nextImg.copyTo(img);
    nextImageWaiting = false;
  }
  else
  {
    printf("!!!! ERROR NO NEW IMAGE !!!!\n"); //should never happen with only 1 thread
    return;
  }
  imgMutex.Unlock();

//#########################################################3
//TODO Check if necessary
  uint64_t ticIMG = get_now();
  // create image pyramid
  img.copyTo(pyramid[0]);
  for (int l = 1; l < LEVELS; l++)
    cv::pyrDown(pyramid[l-1], pyramid[l]);


  uint64_t tocIMG = get_now();
  if (verbose)
    printf("undist/pyr: %6.1f ms\n", (double)(tocIMG - ticIMG) / 1000);
//#########################################################3

#ifdef USE_KLT
  uint64_t ticKLT = get_now();
  //if KLT tracking should be used, this is the moment to track...
  std::vector<unsigned char> status;
  if (!prev_img.empty())
  {
    std::vector<cv::Point2f> curCorners;
    std::vector<float> error;

    cv::calcOpticalFlowPyrLK(prev_img, img, preCorners, curCorners, status, error, cv::Size(32,32), 3);
    preCorners = curCorners;
  }

  imgMutex.Lock();

  if (!prev_img.empty())
  {
    //KLT tracks are on the untouched image, so undistort the current point locations for the filter
    std::vector<cv::Point2f> pointsUndist;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << f, 0, cx, 0, f, cy, 0, 0, 1);
    cv::undistortPoints(preCorners, pointsUndist, cameraMatrix, kc);

    //copy back points and status
    for (int i = 0; i < targetNumPoints; i++)
    {
      points[2*i+0] = f*pointsUndist[i].x+cx;
      points[2*i+1] = f*pointsUndist[i].y+cy;
      cornerStatus[i] = status[i];
    }
  }

  img.copyTo(prev_img);
  imgMutex.Unlock();

  uint64_t tocKLT = get_now();
  if (verbose)
    printf("klt track: %6.1f ms\n", (double)(tocKLT - ticKLT) / 1000);
#endif


  double extractLevel = 0;

  //count valid points and initialize more if necessary
  uint64_t ticI = get_now();
  initMorePoints(img, extractLevel);
  uint64_t tocI = get_now();
  if (verbose)
    printf("init points: %6.1f ms\n", (double)(tocI - ticI) / 1000);

#ifdef MATLAB
//  double useKLT = 0;
//  char pyramid_levels = 4;
//  double anchor_patchSize = 5;
  double iterations = 1;

  double initAnchors[16];

  
//#########################################################
// TODO Check if necessary
  //copy image pyramid (this is only necessary for photometric EKF)
  for (int l = 0; l < 4; l++)
  {
    //first time: allocate space for images
    if (pyramid_col[l].empty())
    {
      pyramid_col[l] = cv::Mat(pyramid[l].cols,pyramid[l].rows,CV_8U);
    }

    //copy column first order
    for (int x = 0; x < pyramid[l].cols; x++)
      for (int y = 0; y < pyramid[l].rows; y++)
        pyramid_col[l].data[x*pyramid[l].rows+y] = pyramid[l].data[y*pyramid[l].cols + x];
  }
//#########################################################

  for (int i = 0; i < numAnchors; i++)
  {
    initAnchors[i] = cornerStatus[i] == 2;

//    printf("%f | %f\n", points[2*i+0], points[2*i+1]);
//    printf("%d ", cornerStatus[i]);
  }
//  printf("\n");

  //outputs
  double cornerStatus_in[numAnchors];
  double cornerStatus_new[numAnchors];
  //debug outputs
  double anchor_u_out[32];
  double anchor_pose_out[112];
  double S_out[64];

  //dummy copy because double datatype...
  for (size_t i = 0; i < numAnchors; i++)
    cornerStatus_in[i] = cornerStatus[i];

  uint64_t tic = get_now();

  //Run EKF update
#ifdef USE_KLT
  SLAM_updIT(P_apr, x_apr, f, cx, cy, cornerStatus_in, points, imNoise, initAnchors, iterations, 5.99, h_u_apo, h_u_apr, x_apo, P_apo, cornerStatus_new, anchor_u_out, anchor_pose_out, S_out);
#else
//  SLAM_updIT2_c(P_apr, x_apr, f, cx, cy, cornerStatus, points, imNoise, numPointsPerAnchor, numAnchors, img_pyr,
//      pyramid_levels, anchor_patchSize, useKLT, initAnchors, iterations, extractLevel,
//      fullAnchorParam, h_u_apo, h_u_apr, x_apo, P_apo, cornerStatus_new);
//  SLAM_updIT2I_c(P_apr, x_apr, f, cx, cy, cornerStatus, points, imNoise, pyramid_col[0].data, pyramid_col[1].data, pyramid_col[2].data, pyramid_col[3].data,
//      useKLT, initAnchors, iterations, extractLevel, fullAnchorParam, h_u_apo, x_apo, P_apo, cornerStatus_new);
#endif

  uint64_t toc = get_now();
  if (verbose)
    printf("Matlab upd: %6.1f ms\n", (double)(toc - tic) / 1000);

#ifndef USE_KLT
  memcpy(points, h_u_apo, sizeof(double)*32);
#endif

#else
  std::vector<unsigned char> cornerStatus_new;
  std::vector<Eigen::Vector2d> np;
  for (int i = 0; i < targetNumPoints; i++)
  {
//    if (cornerStatus[i] == 2)
      np.push_back(Eigen::Vector2d(points[2*i+0], points[2*i+1]));

//    cornerStatus[i] = 1;  //XXX
      cornerStatus_new.push_back(cornerStatus[i]);
  }

  // TODO Remove return to calculate the update
  return;

  uint64_t tic = get_now();
  m_ekf->update(pyramid, imNoise, cornerStatus_new, np);
  uint64_t toc = get_now();
  printf("Update: %6.1f ms\n", (double)(toc - tic) / 1000);
  std::vector<Eigen::Vector2d> p = m_ekf->GetPointPositions();
  for (size_t i = 0; i < p.size(); i++)
  {
    points[2*i+0] = p[i](0);
    points[2*i+1] = p[i](1);
  }
#endif

//  //copy corner status
//  for (int i = 0; i < numAnchors; i++)
//  {
//    if (points[2*i+0] < 0 || points[2*i+0] > pyramid[0].cols || points[2*i+1] < 0 || points[2*i+0] > pyramid[0].rows)
//      cornerStatus[i] = 0;
//    else
//      cornerStatus[i] = cornerStatus_new[i];
//  }


  //TODO uncomment as soon as matlab code is working..
  for (int i = 0; i < targetNumPoints; i++)
  {
    cornerStatus[i] = cornerStatus_new[i];
//    printf("%f | %f\n", h_u_apo[2*i+0], h_u_apo[2*i+1]);
  }

}

static bool sort_func(const std::pair<size_t, size_t>& lhs, const std::pair<size_t, size_t>& rhs)
{
  return lhs.second < rhs.second;
}

void EKFOdometry::initMorePoints(const cv::Mat &img, size_t level)
{
#ifdef MATLAB
  int newAnchorTreshold = 0;  //do not init new points until at least this amount is missing
#else
  int newAnchorTreshold = 5;  //do not init new points until at least this amount is missing
#endif

  //first compute points per grid cell
  std::vector<GridOrder> gridPointCount(gridSizeX*gridSizeY);

  double levelScale = 1. / (1<<level);

  const int gridPixX = (int)img.cols / gridSizeX;
  const int gridPixY = (int)img.rows / gridSizeY;
  const size_t numPointsPerGridCell = (targetNumPoints-1) / gridPointCount.size() + 1;

  //init gridPointCount
  for(int gy = 0; gy < gridSizeY; gy++)
    for(int gx = 0; gx < gridSizeX; gx++)
    {
      gridPointCount[gy*gridSizeX+gx].gx = gx;
      gridPointCount[gy*gridSizeX+gx].gy = gy;
      gridPointCount[gy*gridSizeX+gx].maxScore = 0;
    }

  //compute number of points in grid
  int numPoints = 0;
  for(int i = 0; i < targetNumPoints; i++)
  {
    if (cornerStatus[i] > 0)
    {
      double px = points[2*i+0] * levelScale;
      double py = points[2*i+1] * levelScale;
      int gx = px / gridPixX;
      int gy = py / gridPixY;
      if (gx >= 0 && gx < gridSizeX && gy >= 0 && gy < gridSizeY)
      {
        gridPointCount[gy*gridSizeX+gx].points.push_back(GridOrder::PointWithScore(px, py));
      }
      numPoints++;
    }
  }

  //check if enough points are missing to initialize a new anchor
  if (targetNumPoints - numPoints < newAnchorTreshold)
  {
    return;
  }

  //then sort the grid cells by occupancy
  std::list<std::pair<size_t, size_t> > gridOrder;
  for (size_t i = 0; i < gridPointCount.size(); i++)
  {
    gridOrder.push_back(std::make_pair(i, gridPointCount[i].points.size()));
  }
  gridOrder.sort(sort_func);

  std::vector<cv::Point2f> newPoints;

  int halfpatchAtHighestLevel = halfPatchSize<<(LEVELS-1);

  //now process the grid cells in ascending occupancy order
  for(std::list<std::pair<size_t, size_t> >::iterator it = gridOrder.begin(); it != gridOrder.end(); ++it)
  {
    size_t gridIdx = it->first;
    int gx = gridPointCount[gridIdx].gx;
    int gy = gridPointCount[gridIdx].gy;

    std::vector<GridOrder::PointWithScore> &curPoints = gridPointCount[gridIdx].points;

    //as long as there can be more points try to add more
    if (targetNumPoints > numPoints && curPoints.size() < numPointsPerGridCell)
    {
      //special handling for gridCells at image border
      int to = -5, bo = 5, lo = -5, ro = 5;
      if (gx == 0) lo = halfpatchAtHighestLevel+1;
      if (gy == 0) to = halfpatchAtHighestLevel+1;
      if (gx == gridSizeX-1) ro = -(halfpatchAtHighestLevel+1);
      if (gy == gridSizeY-1) bo = -(halfpatchAtHighestLevel+1);

      int gto = gy*gridPixY+to;
      int gbo = (gy+1)*gridPixY+bo;
      int glo = gx*gridPixX+lo;
      int gro = (gx+1)*gridPixX+ro;

      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack(img.rowRange(gto, gbo).colRange(glo, gro), corners, numPointsPerGridCell*4, 0.03, 10);

      //TODO here sort corners by descending score to first pick the best ones...
      std::vector<std::pair<double, size_t> > score2index(corners.size());
      for (size_t i = 0; i < corners.size(); i++)
      {
        score2index[i] = std::make_pair(shiTomasiScore(img, corners[i], patchSize), i);
      }
      sort(score2index.begin(), score2index.end());

      //now check for every corner if they are valid and useful etc...
      for (size_t ii = 0; ii < corners.size(); ii++)
      {
        size_t i = score2index[ii].second;
        corners[i].x += glo;
        corners[i].y += gto;

        //not near existing corners
        bool valid = true;
        for (int e = 0; e < targetNumPoints; e++)
        {
          if (cornerStatus[e] > 0)
          {
            double dx = points[2*e+0]-corners[i].x;
            double dy = points[2*e+1]-corners[i].y;
            double dist = dx*dx+dy*dy;
            if (dist < minDistSqr * levelScale)
            {
              valid = false;
              break;
            }
          }
        }
        if (valid == true)
        {
          curPoints.push_back(GridOrder::PointWithScore(corners[i].x, corners[i].y));
          newPoints.push_back(cv::Point2f(corners[i].x / levelScale, corners[i].y / levelScale));
          numPoints++;
          if (numPoints == targetNumPoints || curPoints.size() == numPointsPerGridCell)
            break;
        }
      }
    }
  }

  std::vector<cv::Point2f> newPointsUndist;
  if (newPoints.size() > 0)
  {
    //undistort new points
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << f, 0, cx, 0, f, cy, 0, 0, 1);
    undistortPoints(newPoints, newPointsUndist, cameraMatrix, kc);
  }

  //now just add all newly added points to the filter data structures
  for (size_t i = 0; i < newPoints.size(); i++)
  {
    int j = 0;
    //search for the first free position in the points array
    for (; j < targetNumPoints; j++)
    {
      if (cornerStatus[j] == 0)
      {
        cornerStatus[j] = 2;
        preCorners[j] = newPoints[i];
        points[2*j+0] = (f*newPointsUndist[i].x+cx);
        points[2*j+1] = (f*newPointsUndist[i].y+cy);
        break;
      }
    }
    //stop if no more slots for new points
    if (j == targetNumPoints) break;
  }

}

double EKFOdometry::shiTomasiScore(const cv::Mat &image, const cv::Point &position, int halfWindow) const
{
  int iXX = 0;
  int iYY = 0;
  int iXY = 0;

  int x_start = position.x - halfWindow;
  int y_start = position.y - halfWindow;
  int x_end = position.x + halfWindow;
  int y_end = position.y + halfWindow;

  if (x_start <= 0 || y_start <= 0 || x_end >= image.cols || y_end >= image.rows) return 0.;

  for(int y = y_start; y < y_end; y++)
  for(int x = x_start; x < x_end; x++)
  {
    int dx = (int)image.at<unsigned char>(y,x+1) - (int)image.at<unsigned char>(y,x-1);
    int dy = (int)image.at<unsigned char>(y+1,x) - (int)image.at<unsigned char>(y-1,x);
    iXX += dx*dx;
    iYY += dy*dy;
    iXY += dx*dy;
  }

  double nPixels_inv = 1. / (2*(x_end - x_start + 1)*(y_end - y_start + 1));
  double dXX = iXX * nPixels_inv;
  double dYY = iYY * nPixels_inv;
  double dXY = iXY * nPixels_inv;

  // Find and return smaller eigenvalue:
  return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4. * (dXX * dYY - dXY * dXY) ));
}
