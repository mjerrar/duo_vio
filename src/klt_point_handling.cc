#include "klt_point_handling.h"

#include <algorithm>
#include <list>
#include <vector>

#include <iostream>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

//"persistent" local variables
static cv::Mat prev_img;
static std::vector<cv::Point2f> prev_corners;
static std::vector<cv::Point2f> prev_corners_right;
static std::vector<unsigned char> prev_status;

//local functions
static void initMorePoints(const cv::Mat &img, int gridSizeX, int gridSizeY, unsigned int targetNumPoints, double minDistSqr, double minScore);
static double shiTomasiScore(const cv::Mat &image, const cv::Point &position, int halfWindow);

//corners and status are output variables
void handle_points_klt(const cv::Mat &img_l, const cv::Mat &img_r, unsigned int numPoints, double *z_all_l, double *z_all_r, unsigned char *updateVect)
{
  //settings that might or might not be arguments of this function...
  const int gridSizeX = 4;
  const int gridSizeY = 4;
  const unsigned int targetNumPoints = numPoints;
  const double minDistSqr = 10*10;
  const double minScore = 4;

  for (size_t i = 0; i < prev_status.size() && i <numPoints; ++i)
  {
    prev_status.at(i) = updateVect[i];
  }

  std::vector<unsigned char> status;
  std::vector<cv::Point2f> cur_corners;
  std::vector<float> error;

  if (!prev_img.empty())
  {
    cv::calcOpticalFlowPyrLK(prev_img, img_l, prev_corners, cur_corners, status, error, cv::Size(11,11), 3);
    prev_corners = cur_corners;
    for (size_t i = 0; i < prev_status.size() && i <numPoints; ++i)
    {
      prev_status[i] = prev_status[i] && status[i];
    }
  }

  img_l.copyTo(prev_img);

  //check if we need more points
  initMorePoints(img_l, gridSizeX, gridSizeY, targetNumPoints, minDistSqr, minScore);

  if (!img_r.empty())
  {
    std::vector<unsigned char> statusRight;
    cv::calcOpticalFlowPyrLK(img_l, img_r, prev_corners, prev_corners_right, statusRight, error, cv::Size(11,11), 3);

    //compute disparity and write the output variables
    for (size_t i = 0; i < prev_corners.size() && i < numPoints; i++)
    {
      if(status.size() == 0) // status has size 0 the first time
      {
        // only statusRight matters
        if(!(statusRight[i] == 1))
        {
          prev_status[i] = 0;
        }

      } else {
        if(!(statusRight[i] == 1) || (!status[i] == 1 && !prev_status[i] == 2))
        {
          prev_status[i] = 0;
        }
      }

      z_all_l[2*i+0] = prev_corners[i].x;
      z_all_l[2*i+1] = prev_corners[i].y;

      z_all_r[2*i+0] = prev_corners_right[i].x;
      z_all_r[2*i+1] = prev_corners_right[i].y;

      //check disparity
      double diffx = prev_corners[i].x - prev_corners_right[i].x;
      double diffy = prev_corners[i].y - prev_corners_right[i].y;
      double disparity = sqrt(diffx*diffx + diffy*diffy);

//      if (diffx > -3.0 && diffy/diffx < 0.1 && diffx < 250)
//        z_all_l[3*i+2] = disparity;
//      else
//      {
//        z_all_l[3*i+2] = -1000;  //outlier
//      }

        updateVect[i] = prev_status[i];
    }


  } else {
    printf("Right image is empty!\n");
    for (size_t i = 0; i < prev_corners.size() && i < numPoints; i++)
    {
      z_all_l[3*i+2] = -1000;
    }
  }
}

// ==== local functions, hidden from outside this file ====

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

static bool sort_func(const std::pair<size_t, size_t>& lhs, const std::pair<size_t, size_t>& rhs)
{
  return lhs.second < rhs.second;
}

static void initMorePoints(const cv::Mat &img, int gridSizeX, int gridSizeY, unsigned int targetNumPoints, double minDistSqr, double minScore)
{
  unsigned int newAnchorTreshold = 0;  //do not init new points until at least this amount is missing

  //first compute points per grid cell
  std::vector<GridOrder> gridPointCount(gridSizeX*gridSizeY);

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
  unsigned int numPoints = 0;
  for(size_t i = 0; i < prev_status.size(); i++)
  {
    if (prev_status[i] > 0)
    {
      double px = prev_corners[i].x;
      double py = prev_corners[i].y;
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
  if (targetNumPoints - numPoints < newAnchorTreshold) return;

  //then sort the grid cells by occupancy
  std::list<std::pair<size_t, size_t> > gridOrder;
  for (size_t i = 0; i < gridPointCount.size(); i++)
  {
    gridOrder.push_back(std::make_pair(i, gridPointCount[i].points.size()));
  }
  gridOrder.sort(sort_func);

  std::vector<cv::Point2f> newPoints;

  int patchSize = 5;
  int halfpatchSize = patchSize/2;

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
      int to = -halfpatchSize, bo = halfpatchSize, lo = -halfpatchSize, ro = halfpatchSize;
      if (gx == 0) lo = halfpatchSize+1;
      if (gy == 0) to = halfpatchSize+1;
      if (gx == gridSizeX-1) ro = -(halfpatchSize+1);
      if (gy == gridSizeY-1) bo = -(halfpatchSize+1);

      int gto = gy*gridPixY+to;
      int gbo = (gy+1)*gridPixY+bo;
      int glo = gx*gridPixX+lo;
      int gro = (gx+1)*gridPixX+ro;

      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack(img.rowRange(gto, gbo).colRange(glo, gro), corners, numPointsPerGridCell*4, 0.03, 10);

      //sort corners by descending score to first pick the best ones...
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
        for (size_t e = 0; e < prev_status.size(); e++)
        {
          if (prev_status[e] > 0)
          {
            double dx = prev_corners[e].x-corners[i].x;
            double dy = prev_corners[e].y-corners[i].y;
            double dist = dx*dx+dy*dy;
            if (dist < minDistSqr)
            {
              valid = false;
              break;
            }
          }
        }
        if (valid == true)
        {
          curPoints.push_back(GridOrder::PointWithScore(corners[i].x, corners[i].y));
          newPoints.push_back(cv::Point2f(corners[i].x, corners[i].y));
          numPoints++;
          if (numPoints == targetNumPoints || curPoints.size() == numPointsPerGridCell)
            break;
        }
      }
    }
  }

  //now just add all newly added points to the filter data structures
  size_t j = 0;
  for (size_t i = 0; i < newPoints.size(); i++)
  {
    //search for the first free position in the points array
    for (; j < prev_status.size(); j++)
    {
      if (prev_status[j] == 0)
      {
        prev_status[j] = 2;
        prev_corners[j] = newPoints[i];
        i++;
      }
    }
    //stop if no more slots for new points
    if (j == targetNumPoints) break;

    //we prev_corners is full but is smaller than the target size, so we add points to the vector
    prev_status.push_back(2);
    prev_corners.push_back(newPoints[i]);
    prev_corners_right.push_back(newPoints[i]);
    j++;

  }

}

static double shiTomasiScore(const cv::Mat &image, const cv::Point &position, int halfWindow)
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
