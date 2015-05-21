/*
 * EKFPhotometric.h
 *
 *  Created on: Nov 16, 2014
 *      Author: tpetri
 */

#ifndef EKFPhotometric_H_
#define EKFPhotometric_H_

#define EIGEN_USE_MKL_ALL

#include "camera_models/CameraModel.h"

#include <Eigen/Geometry>
#include <list>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <vector>

#define FANCY_P

class MapView;

namespace sfm
{

template <int patchsize, int pyrlevels = 4, int states = 13>
class EKFPhotometric
{
  class PhotometricAnchor
  {
  public:
    struct AnchorFeature
      {
        Eigen::Matrix<double, 1, 1> m_rho;   //depth, this is a state
        Eigen::Matrix<double, 2, 1> m_uv;    //local in image, constant
        Eigen::Matrix<double, 3, 1> m_m;     //ray through uv, constant
        cv::Mat m_patch[pyrlevels];          //extracted patch on each level, size+1
      };

  public:
    void CreateFeatureData(const CameraModel<double> &cam, const cv::Mat image[pyrlevels], std::vector<Eigen::Matrix<double, 2, 1> > &newPoints, double rhoInit)
    {
      int halfpatchm = patchsize / 2;
      int halfpatchp = patchsize / 2 + 1;
      for (size_t i = m_Features.size(); i < newPoints.size(); i++)
      {
        AnchorFeature *f = new AnchorFeature;
        m_Features.push_back(f);
      }
      size_t j = 0;
      typename std::list<AnchorFeature *>::iterator fit;
      for (fit = m_Features.begin(); fit != m_Features.end(); ++fit)
      {
        AnchorFeature *f = *fit;
        f->m_rho(0) = rhoInit;
        f->m_uv = newPoints[j];
        cam.planeToSpace(f->m_uv, f->m_m);
        Eigen::Vector2d uv = newPoints[j];
        for (int i = 0; i < pyrlevels; i++)
        {
          if (f->m_patch[i].empty()) f->m_patch[i] = cv::Mat(patchsize, patchsize, CV_32F);
          //extract reference patch with bilinear filtering
          float fx = uv(0);
          float fy = uv(1);
          const int lx = (int) fx;
          const int ly = (int) fy;
          fx -= lx;
          fy -= ly;
          for(int y = ly-halfpatchm; y < ly+halfpatchp; y++)
          {
            float *rpdata = (float*)f->m_patch[i].row(y-ly+halfpatchm).data;
            for(int x = lx-halfpatchm; x < lx+halfpatchp; x++)
            {
              rpdata[x-lx+halfpatchm] = (1 - fy) * ((1 - fx) * image[i].at<unsigned char>(y,x) 
                + fx * image[i].at<unsigned char>(y,x + 1))
                + fy * ((1 - fx) * image[i].at<unsigned char>(y + 1,x)
                    + fx * image[i].at<unsigned char>(y + 1,x + 1));
            }
          }
          uv *= 0.5;
        }
        j++;
      }
    }

    int GetNumberOfPoints() const { return m_Features.size(); }
    Eigen::Matrix<double, patchsize*patchsize, states> GetDerivWRTCamStates();
    Eigen::Matrix<double, patchsize*patchsize, Eigen::Dynamic> GetDerivWRTAnchorStates();

    friend class MapView;
    friend class EKFPhotometric<patchsize, pyrlevels, states>;
  protected:
    Eigen::Matrix<double, 3, 1> m_frwc;
    Eigen::Matrix<double, 4, 1> m_fqwc;
    std::list<AnchorFeature *> m_Features;
    Eigen::Matrix<double, states, Eigen::Dynamic> m_Pxy;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_Pyy;
    std::list<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > m_PyyList;
  };

  friend class MapView;

public:
  typedef PhotometricAnchor PAnchor;

public:
  EKFPhotometric(const CameraModel<double> &cam) : mCam(cam), m_reset(false) {}
//  EKFPhotometric(double f, double cx, double cy, int patchSize = 9, int gridSizeX = 4, int gridSizeY = 4, int targetNumPoints = 16, double minScore = 50, int minPixDist = 30) : f(f), cx(cx), cy(cy), patchSize(patchSize), halfPatchSize((patchSize+1)/2), gridSizeX(gridSizeX), gridSizeY(gridSizeY), targetNumPoints(targetNumPoints), minScore(minScore), minDistSqr(minPixDist*minPixDist) { cornerStatus = (unsigned char *) malloc(targetNumPoints * sizeof(unsigned char)); memset(cornerStatus, 0, targetNumPoints * sizeof(unsigned char)); gridScore = (double *) malloc(targetNumPoints * sizeof(double)); gridPoint = new cv::Point2f[targetNumPoints]; gridOccupied = (unsigned char *) malloc(targetNumPoints * sizeof(unsigned char)); points = (double *)malloc(2*targetNumPoints*sizeof(double)); /* for (int l = 0; l < 4; l++) img_pyr[l].img = NULL;*/}

  void Reset()
  {
    init(Eigen::Isometry3d::Identity());
    m_Anchors.clear();
    m_reset = true;
  }

public:
  void init(const Eigen::Isometry3d &cameraPosition);
  void predict(double dt);
  void update(const cv::Mat image[pyrlevels], double imNoise, std::vector<unsigned char> &cornerStatus, std::vector<Eigen::Vector2d> &newPoints);

public:
  Eigen::Isometry3d GetPosition() const
  {
#ifdef FANCY_P
    Eigen::Vector3d r_wc = m_camStates.template head<3>();
    Eigen::Vector4d q_wc(m_camStates.template segment<4>(3));
#else
    Eigen::Vector3d r_wc = m_x.template head<3>();
    Eigen::Vector4d q_wc(m_x.template segment<4>(3));
#endif

    Eigen::Matrix3d R_wc;
    R_wc << q_wc(0)*q_wc(0)+q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(1)*q_wc(2)-q_wc(0)*q_wc(3)) , 2.*(q_wc(3)*q_wc(1)+q_wc(0)*q_wc(2)),
           2.*(q_wc(1)*q_wc(2)+q_wc(0)*q_wc(3)) , q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)+q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(2)*q_wc(3)-q_wc(0)*q_wc(1)),
           2.*(q_wc(3)*q_wc(1)-q_wc(0)*q_wc(2)) , 2.*(q_wc(2)*q_wc(3)+q_wc(0)*q_wc(1)), q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)+q_wc(3)*q_wc(3);

    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
    ret.linear() = R_wc.transpose();
    ret.translation() = -R_wc.transpose() * r_wc;
    return ret;
  }

  std::vector<Eigen::Vector2d> GetPointPositions() const { return m_last_projections; }

protected:
  void RemovePoint();
  void RemoveAnchor();
  int GetTotalNumberOfStates();
  void BuildStateAndCovariance(Eigen::Matrix<double, Eigen::Dynamic,1> &x, Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &P);
  void StoreStateAndCovariance(const Eigen::Matrix<double, Eigen::Dynamic,1> &x, const Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &P);
  int GetWarpMatrixAndLevel(double depth, const Eigen::Vector2d &ref_uv, int ref_level, const Eigen::Matrix3d &R_cw, const Eigen::Vector3d &r_wc, const Eigen::Matrix3d &fR_wc, const Eigen::Vector3d &fr_wc, Eigen::Matrix2d &A);
  bool GetPredictedPatch(const Eigen::Matrix2d &A, const cv::Mat &im, const Eigen::Vector2d &ref_pixelPos, int search_level, cv::Mat &patch);

protected:
  const CameraModel<double> &mCam;

  //states
#ifdef FANCY_P
  Eigen::Matrix<double, states, 1> m_camStates;
  Eigen::Matrix<double, states, states> m_Pxx;
#else
  Eigen::Matrix<double, Eigen::Dynamic, 1> m_x;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_P;
#endif
  std::list<PAnchor *> m_Anchors;

  Eigen::Matrix<double, states, states> m_F;
  Eigen::Matrix<double, states, 6> m_G;
  Eigen::Matrix<double, 6, 1> m_Qxt;

  std::vector<Eigen::Vector2d> m_last_projections;
  bool m_reset;
};

}

#include "EKFPhotometricImpl.h"

#endif /* EKFPhotometric_H_ */
