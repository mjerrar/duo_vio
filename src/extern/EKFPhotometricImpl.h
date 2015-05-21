#include <iostream>
#include <iomanip>
#include <fstream>

//#define REMOVE_OUT_OF_VIEW_POINTS

namespace sfm
{

template <int patchsize, int pyrlevels, int states>
void EKFPhotometric<patchsize, pyrlevels, states>::init(const Eigen::Isometry3d &cameraPosition)
{
//  Eigen::Matrix<double, states, 1> test = m_camStatesApo.template head<states>();
  Eigen::Quaterniond quat(cameraPosition.linear().transpose());

#ifdef FANCY_P
  m_camStates = Eigen::Matrix<double, states, 1>::Zero();
//  m_camStates.template head<3>() = cameraPosition.inverse(Eigen::Isometry).translation();
  m_camStates.template segment<4>(3) = Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
  m_Pxx = Eigen::Matrix<double, 13, 13>::Ones();
#else
  m_x = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(states);
  m_x.segment(3,4) = Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
  m_P = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Ones(states, states);
#endif

  m_F = Eigen::Matrix<double, 13, 13>::Identity();
  m_G = Eigen::Matrix<double, 13, 6>::Zero();
  m_G.template block<3,3>(7,0) = Eigen::Matrix3d::Identity();
  m_G.template block<3,3>(10,3) = Eigen::Matrix3d::Identity();

  double qv = 1;
  double qw = 0.1;

  m_Qxt << qv, qv, qv, qw, qw, qw;
}

template <int patchsize, int pyrlevels, int states>
void EKFPhotometric<patchsize, pyrlevels, states>::predict(double dt)
{
  if (m_reset) return;

#ifdef FANCY_P
  const Eigen::Vector3d &r_wc = m_camStates.template head<3>();
  const Eigen::Vector4d &q_wc(m_camStates.template segment<4>(3));
  const Eigen::Vector3d &r_v = m_camStates.template segment<3>(7);
  const Eigen::Vector3d &w_c = m_camStates.template segment<3>(10);
#else
  const Eigen::Vector3d &r_wc = m_x.template head<3>();
  const Eigen::Vector4d &q_wc (m_x.template segment<4>(3));
  const Eigen::Vector3d &r_v = m_x.template segment<3>(7);
  const Eigen::Vector3d &w_c = m_x.template segment<3>(10);
#endif

  //=============================================================

  //integrate gyro
  Eigen::Vector3d w_c_dt = w_c * dt;
  double theta = w_c_dt.norm();
  if (theta != 0) w_c_dt /= theta;

  double sintheta05 = sin(theta*0.5);
  Eigen::Vector4d dq(cos(theta*0.5), w_c_dt(0)*sintheta05, w_c_dt(1)*sintheta05, w_c_dt(2)*sintheta05);

  //q1 is dq - the delta rotation during dt
  //q2 is rotation at time k-1
  //q3 is rotation at time k

  const Eigen::Vector4d &q1 = dq;
  Eigen::Matrix4d dq3Todq2;
  dq3Todq2 << q1(0), -q1(1), -q1(2), -q1(3),
              q1(1),  q1(0),  q1(3), -q1(2),
              q1(2), -q1(3),  q1(0),  q1(1),
              q1(3),  q1(2), -q1(1),  q1(0);

  const Eigen::Vector4d &q2 = q_wc;
  Eigen::Matrix4d dq3Todq1;// = Eigen::Matrix4d::Identity();
  dq3Todq1 << q2(0), -q2(1), -q2(2), -q2(3),
              q2(1),  q2(0), -q2(3),  q2(2),
              q2(2),  q2(3),  q2(0), -q2(1),
              q2(3), -q2(2),  q2(1),  q2(0);

  Eigen::Matrix<double, 4, 3> dq1Todw = Eigen::Matrix<double, 4, 3>::Zero();
  if (theta != 0.)
  {
    double t01= -(dt*w_c(0)/(2*theta)*sin((dt*theta)*0.5));
    double t02= -(dt*w_c(1)/(2*theta)*sin((dt*theta)*0.5));
    double t03= -(dt*w_c(2)/(2*theta)*sin((dt*theta)*0.5));
    double t12= (w_c(1)*w_c(0)/(theta*theta))*(dt*0.5*cos(theta*dt*0.5)-1/theta*sin(theta*dt*0.5));
    double t13= (w_c(2)*w_c(0)/(theta*theta))*(dt*0.5*cos(theta*dt*0.5)-1/theta*sin(theta*dt*0.5));
    double t23= (w_c(2)*w_c(1)/(theta*theta))*(dt*0.5*cos(theta*dt*0.5)-1/theta*sin(theta*dt*0.5));
    double t11= dt*0.5*(w_c(0)/theta)*(w_c(0)/theta)*cos(theta*dt*0.5)+1/theta*sin(theta*dt*0.5)*(1-(w_c(0)/theta)*(w_c(0)/theta));
    double t22= dt*0.5*(w_c(1)/theta)*(w_c(1)/theta)*cos(theta*dt*0.5)+1/theta*sin(theta*dt*0.5)*(1-(w_c(1)/theta)*(w_c(1)/theta));
    double t33= dt*0.5*(w_c(2)/theta)*(w_c(2)/theta)*cos(theta*dt*0.5)+1/theta*sin(theta*dt*0.5)*(1-(w_c(2)/theta)*(w_c(2)/theta));
    dq1Todw << t01,t02,t03,
               t11,t12,t13,
               t12,t22,t23,
               t13,t23,t33;
  }
  else
  {
    dq1Todw.block<3,3>(1,0) = Eigen::Matrix3d::Identity()*(0.5*dt);
  }

  m_F.template block<3,3>(0,7) = dt*Eigen::Matrix3d::Identity();
  m_F.template block<4,4>(3,3) = dq3Todq2;
  m_F.template block<4,3>(3,10) = dq3Todq1*dq1Todw;

  m_G.template block<3,3>(0,0) = Eigen::Matrix3d::Identity()*dt;
  m_G.template block<4,3>(3,3) = dq3Todq1*dq1Todw;

  //=============================================================

#ifdef FANCY_P
  //integrate position
  m_camStates.template head<3>() = r_wc + r_v*dt;

//  //integrate orientation
  Eigen::Vector4d q_wcXdq = Eigen::Vector4d(q_wc(0)*dq(0) - q_wc(1)*dq(1) - q_wc(2)*dq(2) - q_wc(3)*dq(3),
                                            q_wc(0)*dq(1) + q_wc(1)*dq(0) + q_wc(2)*dq(3) - q_wc(3)*dq(2),
                                            q_wc(0)*dq(2) - q_wc(1)*dq(3) + q_wc(2)*dq(0) + q_wc(3)*dq(1),
                                            q_wc(0)*dq(3) + q_wc(1)*dq(2) - q_wc(2)*dq(1) + q_wc(3)*dq(0));
//  double atmp = q_wc(0);
//  Eigen::Vector3d vtmp
//  double xtmp = dq(0);
//  Eigen::Vector4d q_wcXdq
  m_camStates.template segment<4>(3) = q_wcXdq;

  m_Pxx = m_F * m_Pxx * m_F.transpose() + m_G * (m_Qxt.cwiseProduct(m_Qxt)*(dt*dt)).asDiagonal() * m_G.transpose();
  typename std::list<PAnchor *>::iterator ait;
  for (ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
  {
    (*ait)->m_Pxy = m_F * (*ait)->m_Pxy;
  }
#else
  //===============================
  //integrate position
  m_x.template head<3>() = r_wc + r_v*dt;

  //integrate orientation
//  Eigen::Vector4d q_wcXdq = Eigen::Vector4d(q_wc(0)*dq(0) - q_wc(1)*dq(1) - q_wc(2)*dq(2) - q_wc(3)*dq(3),
//                                            q_wc(0)*dq(1) + q_wc(1)*dq(0) + q_wc(2)*dq(3) - q_wc(3)*dq(2),
//                                            q_wc(0)*dq(2) - q_wc(1)*dq(3) + q_wc(2)*dq(0) + q_wc(3)*dq(1),
//                                            q_wc(0)*dq(3) + q_wc(1)*dq(2) - q_wc(2)*dq(1) + q_wc(3)*dq(0));
  Eigen::Vector4d q_wcXdq = Eigen::Vector4d(q_wc(0)*dq(0) - q_wc(1)*dq(1) - q_wc(2)*dq(2) - q_wc(3)*dq(3),
                                            q_wc(0)*dq(1) + q_wc(1)*dq(0) - q_wc(2)*dq(3) + q_wc(3)*dq(2),
                                            q_wc(0)*dq(2) + q_wc(1)*dq(3) + q_wc(2)*dq(0) - q_wc(3)*dq(1),
                                            q_wc(0)*dq(3) - q_wc(1)*dq(2) + q_wc(2)*dq(1) + q_wc(3)*dq(0));

  m_x.template segment<4>(3) = q_wcXdq;

  m_P.block(0,0,states, states) = m_F * m_P.block(0,0,states, states) * m_F.transpose() + m_G * (m_Qxt.cwiseProduct(m_Qxt)*(dt*dt)).asDiagonal() * m_G.transpose();
  if (m_P.cols() > states)
  {
    m_P.block(0,states,states, m_P.cols()-states) = m_F * m_P.block(0,states,states, m_P.cols()-states);
    m_P.block(states,0,m_P.rows()-states, states) = m_P.block(states,0, m_P.rows()-states,states) * m_F.transpose();
  }
#endif
}

//#define SHOW_PATCH_DEBUG

template <int patchsize, int pyrlevels, int states>
void EKFPhotometric<patchsize, pyrlevels, states>::update(const cv::Mat image[pyrlevels], double imNoise, std::vector<unsigned char> &cornerStatus, std::vector<Eigen::Vector2d> &newPoints)
{
  if (m_reset)
  {
    for (size_t i = 0; i < cornerStatus.size(); i++)
      cornerStatus[i] = 0;
    m_reset = false;
    return;
  }

  int totalStates = GetTotalNumberOfStates();
  Eigen::Matrix<double, patchsize*patchsize, Eigen::Dynamic> H;
  Eigen::Matrix<double, patchsize*patchsize, 1> res;

#ifdef FANCY_P
  //before starting update, create full state and covariance matrix from data
  Eigen::Matrix<double, Eigen::Dynamic, 1> x;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
  BuildStateAndCovariance(x, P);
#else
  Eigen::Matrix<double, Eigen::Dynamic, 1> &x = m_x;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &P = m_P;
#endif

#ifdef SHOW_PATCH_DEBUG
  int patchscale = 10;
  cv::Mat patch_debug(4*patchsize*patchscale, 4*patchsize*patchscale, CV_8U);
  cv::Mat refpt_debug(4*patchsize*patchscale, 4*patchsize*patchscale, CV_8U);
//  cv::Mat gradx_debug(4*patchsize*patchscale, 4*patchsize*patchscale, CV_8U);
//  cv::Mat grady_debug(4*patchsize*patchscale, 4*patchsize*patchscale, CV_8U);
//  cv::Mat resid_debug(4*patchsize*patchscale, 4*patchsize*patchscale, CV_8U);
  patch_debug.setTo(0);
  refpt_debug.setTo(0);
//  gradx_debug.setTo(0);
//  grady_debug.setTo(0);
//  resid_debug.setTo(0);
#endif

  const int iterations = 1;
  for (int iters = 0; iters < iterations; iters++)
  {

  //iterate over all anchors
  int featureOffset = -1;
  int stateOffset = states;
  typename std::list<PAnchor *>::iterator ait;
  for (ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
  {
    PAnchor &cur_anchor = *(*ait);

    //offset counter logic magic...
    if (featureOffset == -1) stateOffset -= 7+cur_anchor.GetNumberOfPoints();
    stateOffset += 7+cur_anchor.GetNumberOfPoints();

    Eigen::Matrix<double, 3, Eigen::Dynamic> J_ra;

    //iterate over all points per anchor
    int localFeatureCounter = -1;
    typename std::list<typename PAnchor::AnchorFeature *>::iterator pit;
    for (pit = cur_anchor.m_Features.begin(); pit != cur_anchor.m_Features.end(); ++pit)
    {
      featureOffset++;
      localFeatureCounter++;

//      printf("stateoffset %d, feature %d, %d\n", stateOffset, featureOffset, localFeatureCounter);

      if (cornerStatus[featureOffset] != 1) continue;

      typename PAnchor::AnchorFeature &af = *(*pit);

      //set H always to zero for every feature since only some parts of it are set
      H = H.Zero(patchsize*patchsize, totalStates);
      J_ra = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, 7 + cur_anchor.GetNumberOfPoints());

      //=========================================================
      // copy fresh data into the local variables, has to be done for every feature because of iterated KF
      //=========================================================
      const Eigen::Vector3d &r_wc = x.template head<3>();
      const Eigen::Vector4d &q_wc(x.template segment<4>(3));
      const Eigen::Vector3d &fr_wc = x.template segment<3>(stateOffset+0);
      const Eigen::Vector4d &fq_wc = x.template segment<4>(stateOffset+3);
      double rho = x(stateOffset+7+localFeatureCounter);

//      std::cout << r_wc.transpose() << std::endl;
//      std::cout << q_wc.transpose() << std::endl;
//      std::cout << fr_wc.transpose() << std::endl;
//      std::cout << fq_wc.transpose() << std::endl;
//      std::cout << rho << std::endl;

      Eigen::Matrix3d R_wc;
      R_wc << q_wc(0)*q_wc(0)+q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(1)*q_wc(2)-q_wc(0)*q_wc(3)) , 2.*(q_wc(3)*q_wc(1)+q_wc(0)*q_wc(2)),
              2.*(q_wc(1)*q_wc(2)+q_wc(0)*q_wc(3)) , q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)+q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(2)*q_wc(3)-q_wc(0)*q_wc(1)),
              2.*(q_wc(3)*q_wc(1)-q_wc(0)*q_wc(2)) , 2.*(q_wc(2)*q_wc(3)+q_wc(0)*q_wc(1)), q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)+q_wc(3)*q_wc(3);

      Eigen::Matrix3d R_cw(R_wc.transpose());
      Eigen::Vector4d q_cw(q_wc(0), -q_wc(1), -q_wc(2), -q_wc(3));

      //jacobian wrt camera orientation
      Eigen::Matrix<double,3,3> R_cw2q_wc0, R_cw2q_wc1, R_cw2q_wc2, R_cw2q_wc3;
      R_cw2q_wc0 <<  q_cw(0), -q_cw(3),  q_cw(2),
                     q_cw(3),  q_cw(0), -q_cw(1),
                    -q_cw(2),  q_cw(1),  q_cw(0);

      R_cw2q_wc1 <<  q_cw(1),  q_cw(2),  q_cw(3),
                     q_cw(2), -q_cw(1), -q_cw(0),
                     q_cw(3),  q_cw(0), -q_cw(1);

      R_cw2q_wc2 << -q_cw(2),  q_cw(1),  q_cw(0),
                     q_cw(1),  q_cw(2),  q_cw(3),
                    -q_cw(0),  q_cw(3), -q_cw(2);

      R_cw2q_wc3 << -q_cw(3), -q_cw(0),  q_cw(1),
                     q_cw(0), -q_cw(3),  q_cw(2),
                     q_cw(1),  q_cw(2),  q_cw(3);

      //read anchor orientation
      Eigen::Matrix3d fR_wc, fR_cw2q_wc0, fR_cw2q_wc1, fR_cw2q_wc2, fR_cw2q_wc3;
      {
        const Eigen::Vector4d &q_wc = fq_wc;
        fR_wc << q_wc(0)*q_wc(0)+q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(1)*q_wc(2)-q_wc(0)*q_wc(3)) , 2.*(q_wc(3)*q_wc(1)+q_wc(0)*q_wc(2)),
                  2.*(q_wc(1)*q_wc(2)+q_wc(0)*q_wc(3)) , q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)+q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(2)*q_wc(3)-q_wc(0)*q_wc(1)),
                  2.*(q_wc(3)*q_wc(1)-q_wc(0)*q_wc(2)) , 2.*(q_wc(2)*q_wc(3)+q_wc(0)*q_wc(1)), q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)+q_wc(3)*q_wc(3);

        const Eigen::Vector4d &q_cw = fq_wc;//Eigen::Vector4d(fq_wc(0),-fq_wc(1),-fq_wc(2),-fq_wc(3));
        fR_cw2q_wc0 <<  q_cw(0), -q_cw(3),  q_cw(2),
                        q_cw(3),  q_cw(0), -q_cw(1),
                       -q_cw(2),  q_cw(1),  q_cw(0);

        fR_cw2q_wc1 <<  q_cw(1),  q_cw(2),  q_cw(3),
                        q_cw(2), -q_cw(1), -q_cw(0),
                        q_cw(3),  q_cw(0), -q_cw(1);

        fR_cw2q_wc2 << -q_cw(2),  q_cw(1),  q_cw(0),
                        q_cw(1),  q_cw(2),  q_cw(3),
                       -q_cw(0),  q_cw(3), -q_cw(2);

        fR_cw2q_wc3 << -q_cw(3), -q_cw(0),  q_cw(1),
                        q_cw(0), -q_cw(3),  q_cw(2),
                        q_cw(1),  q_cw(2),  q_cw(3);
      }
      //=========================================================
      //=========================================================

      const Eigen::Vector3d &m = af.m_m;

      //project current point into the current camera and compute derivatives
      Eigen::Vector3d h_wi = (rho * (fr_wc - r_wc) + fR_wc*m);
      Eigen::Vector3d h_ci = R_cw * h_wi;
      Eigen::Vector2d h_ui;
      Eigen::Matrix<double, 2, 3> Jproj;
      mCam.spaceToPlane(h_ci, h_ui, Jproj);

      Eigen::Matrix<double, 3, 7> Jrx;
      Jrx.block<3,3>(0,0) = -rho * R_cw;
      Jrx.col(3) =  2. * (R_cw2q_wc0 * h_wi);
      Jrx.col(4) = -2. * (R_cw2q_wc1 * h_wi);
      Jrx.col(5) = -2. * (R_cw2q_wc2 * h_wi);
      Jrx.col(6) = -2. * (R_cw2q_wc3 * h_wi);

      J_ra.block<3,3>(0,0) = rho * R_cw;
      J_ra.col(3) = 2. * (R_cw*(fR_cw2q_wc0 * m));
      J_ra.col(4) = 2. * (R_cw*(fR_cw2q_wc1 * m));
      J_ra.col(5) = 2. * (R_cw*(fR_cw2q_wc2 * m));
      J_ra.col(6) = 2. * (R_cw*(fR_cw2q_wc3 * m));
      J_ra.col(7+localFeatureCounter) = R_cw*(fr_wc - r_wc);

      //now do the fancy pixel patch computations...
      //first, warp the patch around the current point projection into the anchor view
      Eigen::Matrix2d A;
      int ref_level = 3;
      int search_level = GetWarpMatrixAndLevel(1/rho, af.m_uv, ref_level, R_cw, r_wc, fR_wc, fr_wc, A);
      if (search_level < 0)
      {
#ifdef REMOVE_OUT_OF_VIEW_POINTS
        cornerStatus[featureOffset] = 0;
#endif
        printf("invalid warp\n");
        continue;
      }

      //TODO fix level selection

//      printf("searchlevel=%d\n", search_level);
      //warp patch from a reasonable level
      cv::Mat warped_patch_with_border;
      bool good = GetPredictedPatch(A, image[search_level], h_ui, search_level, warped_patch_with_border);
      if (!good)
      {
#ifdef REMOVE_OUT_OF_VIEW_POINTS
        cornerStatus[featureOffset] = 0;
#endif
        printf("invalid warped patch\n");
        continue;
      }

      //compute image gradients with Sobel
      cv::Mat Ix_with_border, Iy_with_border;
      cv::Sobel(warped_patch_with_border, Ix_with_border, CV_32F, 1, 0, 3, 0.125);
      cv::Sobel(warped_patch_with_border, Iy_with_border, CV_32F, 0, 1, 3, 0.125);

#ifdef SHOW_PATCH_DEBUG
      cv::Mat curPatchDebug;
      cv::Mat refPatchDebug;
      cv::Mat curResidDebug;
      cv::Mat rtmp;
      if (iters == iterations-1)
      {
        cv::Mat tmp;
        int py = featureOffset / 4;
        int px = featureOffset % 4;
        curPatchDebug = patch_debug.rowRange(py * patch_debug.rows/4, (py+1)*patch_debug.rows/4).colRange(px * patch_debug.cols/4, (px+1) * patch_debug.cols/4);
        cv::resize(warped_patch_with_border.rowRange(1, patchsize+1).colRange(1, patchsize+1), tmp, cv::Size(curPatchDebug.cols, curPatchDebug.rows));
        tmp.convertTo(curPatchDebug, CV_8U);
  //      tmp.copyTo(curPatchDebug);
        refPatchDebug = refpt_debug.rowRange(py * refpt_debug.rows/4, (py+1)*refpt_debug.rows/4).colRange(px * refpt_debug.cols/4, (px+1) * refpt_debug.cols/4);
        cv::resize(af.m_patch[search_level].rowRange(0, patchsize).colRange(0, patchsize), tmp, cv::Size(curPatchDebug.cols, curPatchDebug.rows));
        tmp.convertTo(refPatchDebug, CV_8U);
  //      tmp.copyTo(refPatchDebug);
  //      cv::Mat curgradxDebug = gradx_debug.rowRange(py * gradx_debug.rows/4, (py+1)*gradx_debug.rows/4).colRange(px * gradx_debug.cols/4, (px+1) * gradx_debug.cols/4);
  //      cv::resize(Ix_with_border.rowRange(1, patchsize+1).colRange(1, patchsize+1), tmp, cv::Size(curgradxDebug.cols, curgradxDebug.rows));
  //      tmp.convertTo(curgradxDebug, CV_8U, 0.5, 128);
  //      cv::Mat curgradyDebug = grady_debug.rowRange(py * grady_debug.rows/4, (py+1)*gradx_debug.rows/4).colRange(px * grady_debug.cols/4, (px+1) * grady_debug.cols/4);
  //      cv::resize(Iy_with_border.rowRange(1, patchsize+1).colRange(1, patchsize+1), tmp, cv::Size(curgradyDebug.cols, curgradyDebug.rows));
  //      tmp.convertTo(curgradyDebug, CV_8U, 0.5, 128);
//        curResidDebug = resid_debug.rowRange(py * resid_debug.rows/4, (py+1)*resid_debug.rows/4).colRange(px * resid_debug.cols/4, (px+1) * resid_debug.cols/4);
//        rtmp = cv::Mat(patchsize, patchsize, CV_8U);
      }
#endif
//      printf("searchlevel=%d\n", search_level);
      //create H entry
      Jproj /= 1<<search_level;
      Eigen::Matrix<double, 2, 7> J_x = Jproj * Jrx;
      Eigen::Matrix<double, 2, Eigen::Dynamic> J_a = Jproj * J_ra;

      bool bRobust = false;
      double threshold = 30;
      double weight = 1.;

      //now for every pixel, create H row and compute residual
      int c = 0;
      for(int y = 1; y < patchsize+1; y++)
      {
        float *Ixdata = (float *)(&Ix_with_border.data[y*Ix_with_border.step[0]]);
        float *Iydata = (float *)(&Iy_with_border.data[y*Iy_with_border.step[0]]);
        float *cpdata = (float *)warped_patch_with_border.row(y).data;
        float *rpdata = (float *)af.m_patch[search_level].row(y-1).data;

        for(int x = 1; x < patchsize+1; x++)
        {
          if (bRobust)
            weight = sqrt(1.0 / (1.0 + fabs(rpdata[x-1] - cpdata[x]) / threshold));

          res(c) = weight * (rpdata[x-1] - cpdata[x]);
          Eigen::Vector2d dI(weight * Ixdata[x], weight * Iydata[x]);

          H.row(c).template head<7>() = (dI).transpose() * A.inverse() * J_x;
          H.row(c).segment(stateOffset, J_a.cols()) = (dI).transpose() * A.inverse() * J_a;
          c++;
        }
      }
//#ifdef SHOW_PATCH_DEBUG
//      if (iters == iterations-1)
//      {
//        int c = 0;
//        for(int y = 1; y < patchsize+1; y++)
//        {
//          for(int x = 1; x < patchsize+1; x++)
//          {
//            rtmp.at<unsigned char>(y-1, x-1) = res(c)*.5+128;
//            c++;
//          }
//        }
//        cv::Mat tmp;
//        cv::resize(rtmp, tmp, cv::Size(curResidDebug.cols, curResidDebug.rows));
//        tmp.copyTo(curResidDebug);
//      }
//#endif

      //do hardcoded ssd error check
      double sqrtErrThres = patchsize*patchsize*10;
      if (res.transpose()*res > sqrtErrThres*sqrtErrThres)
      {
#ifdef REMOVE_OUT_OF_VIEW_POINTS
        cornerStatus[featureOffset] = 0;
#endif
        printf("ssd outlier\n");
        continue;
      }

      //do sequential update for the current feature / patch
      Eigen::Matrix<double, Eigen::Dynamic, patchsize*patchsize> PHT = P * H.transpose();
      Eigen::Matrix<double, patchsize*patchsize, patchsize*patchsize> S = H * PHT + ((imNoise*imNoise) * Eigen::Matrix<double, patchsize*patchsize, patchsize*patchsize>::Identity());
      Eigen::Matrix<double, Eigen::Dynamic, patchsize*patchsize> K = S.transpose().ldlt().solve(PHT.transpose()).transpose();
//      Eigen::Matrix<double, Eigen::Dynamic, patchsize*patchsize> K = PHT * S.inverse();
//      printf("\n\nFeature %d\n\n", featureOffset);
//      std::cout << J_x << std::endl;
//      std::cout << J_a << std::endl;
//      std::cout << H << std::endl;
//      std::cout << "res=" << res.transpose() << std::endl;

      x += K*res;
      P -= K*H*P;
//      P -= K * S * K.transpose();
    }
  }

  } //iterations

#ifdef SHOW_PATCH_DEBUG
  cv::imshow("Patches Current", patch_debug);
  cv::imshow("Patches Reference", refpt_debug);
  //  cv::imshow("X Gradient", gradx_debug);
  //  cv::imshow("Y Gradient", grady_debug);
//  cv::imshow("Residual", resid_debug);
  cv::moveWindow("Patches Current", 1920+1300, 50);
  cv::moveWindow("Patches Reference", 1920+1300+4*patchsize*10+2, 50);
//  cv::moveWindow("Residual", 2*4*patchsize*10, 0);
  cv::moveWindow("Camera", 1920, 50);
#endif

//  std::cout << m_camStatesApo.transpose() << std::endl;

#ifdef FANCY_P
  //after all updates are done, store P matrix into data
  StoreStateAndCovariance(x, P);

  int stateOffset;
  //normalization of the quaternion
  Eigen::Vector4d q(m_camStates.template segment<4>(3));
  //compute derivative of normalization
  Eigen::Matrix4d q_norm_To_q;
  q_norm_To_q << q(1)*q(1)+q(2)*q(2)+q(3)*q(3) , -q(0)*q(1)                    , -q(0)*q(2)                    , -q(0)*q(3),
                 -q(1)*q(0)                    , q(0)*q(0)+q(2)*q(2)+q(3)*q(3) , -q(1)*q(2)                    , -q(1)*q(3),
                 -q(2)*q(0)                    , -q(2)*q(1)                    , q(0)*q(0)+q(1)*q(1)+q(3)*q(3) , -q(2)*q(3),
                 -q(3)*q(0)                    , -q(3)*q(1)                    , -q(3)*q(2) , q(0)*q(0)+q(1)*q(1)+q(2)*q(2);
  double q_norm = q.norm();
  q_norm_To_q *=  1. / sqrt(q_norm*q_norm*q_norm);

  //and store back
  m_camStates.template segment<4>(3).normalize();
  m_Pxx.template block<4,13>(3,0) = q_norm_To_q * m_Pxx.template block<4,13>(3,0);
  m_Pxx.template block<13,4>(0,3) = m_Pxx.template block<13,4>(0,3) * q_norm_To_q.transpose();

  // also loop through all Pxy and normalize them too
  for (typename std::list<PAnchor *>::iterator ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
  {
    PAnchor &cur_anchor = *(*ait);
    cur_anchor.m_Pxy.template block<4,7>(3,0) = q_norm_To_q * cur_anchor.m_Pxy.template block<4,7>(3,0);
  }

  //and also normalize its own quaternion
  for (typename std::list<PAnchor *>::iterator ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
  {
    PAnchor &cur_anchor = *(*ait);
    Eigen::Vector4d q(cur_anchor.m_fqwc);
    q_norm_To_q << q(1)*q(1)+q(2)*q(2)+q(3)*q(3) , -q(0)*q(1)                    , -q(0)*q(2)                    , -q(0)*q(3),
                   -q(1)*q(0)                    , q(0)*q(0)+q(2)*q(2)+q(3)*q(3) , -q(1)*q(2)                    , -q(1)*q(3),
                   -q(2)*q(0)                    , -q(2)*q(1)                    , q(0)*q(0)+q(1)*q(1)+q(3)*q(3) , -q(2)*q(3),
                   -q(3)*q(0)                    , -q(3)*q(1)                    , -q(3)*q(2) , q(0)*q(0)+q(1)*q(1)+q(2)*q(2);
    double q_norm = q.norm();
    q_norm_To_q *=  1. / sqrt(q_norm*q_norm*q_norm);
    cur_anchor.m_fqwc.normalize();

//    m_Pxx.template block<4,13>(3,0) = q_norm_To_q * m_Pxx.template block<4,13>(3,0);
//    m_Pxx.template block<13,4>(0,3) = m_Pxx.template block<13,4>(0,3) * q_norm_To_q.transpose();

//    cur_anchor.m_Pxy.template block<13,4>(0,3) = cur_anchor.m_Pxy.template block<13,4>(0,3) * q_norm_To_q.transpose();
  }
#else
  //normalization of the quaternion
  Eigen::Vector4d q(m_x.template segment<4>(3));
  //compute derivative of normalization
  Eigen::Matrix4d q_norm_To_q;
  q_norm_To_q << q(1)*q(1)+q(2)*q(2)+q(3)*q(3) , -q(0)*q(1)                    , -q(0)*q(2)                    , -q(0)*q(3),
                 -q(1)*q(0)                    , q(0)*q(0)+q(2)*q(2)+q(3)*q(3) , -q(1)*q(2)                    , -q(1)*q(3),
                 -q(2)*q(0)                    , -q(2)*q(1)                    , q(0)*q(0)+q(1)*q(1)+q(3)*q(3) , -q(2)*q(3),
                 -q(3)*q(0)                    , -q(3)*q(1)                    , -q(3)*q(2) , q(0)*q(0)+q(1)*q(1)+q(2)*q(2);
  double q_norm = q.norm();
  q_norm_To_q *=  1. / sqrt(q_norm*q_norm*q_norm);

  //and store back
//  m_x.template segment<4>(3).normalize();
//  m_P.block(3,0,4,totalStates) = q_norm_To_q * m_P.block(3,0,4,totalStates);
//  m_P.block(0,3,totalStates,4) = m_P.block(0,3,totalStates,4) * q_norm_To_q.transpose();

  //and also normalize its own quaternion
  int stateOffset = states;
  for (typename std::list<PAnchor *>::iterator ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
  {
//    Eigen::Vector4d q(m_x.template segment<4>(stateOffset+3));
//    q_norm_To_q << q(1)*q(1)+q(2)*q(2)+q(3)*q(3) , -q(0)*q(1)                    , -q(0)*q(2)                    , -q(0)*q(3),
//                   -q(1)*q(0)                    , q(0)*q(0)+q(2)*q(2)+q(3)*q(3) , -q(1)*q(2)                    , -q(1)*q(3),
//                   -q(2)*q(0)                    , -q(2)*q(1)                    , q(0)*q(0)+q(1)*q(1)+q(3)*q(3) , -q(2)*q(3),
//                   -q(3)*q(0)                    , -q(3)*q(1)                    , -q(3)*q(2) , q(0)*q(0)+q(1)*q(1)+q(2)*q(2);
//    double q_norm = q.norm();
//    q_norm_To_q *=  1. / sqrt(q_norm*q_norm*q_norm);
    m_x.template segment<4>(stateOffset+3).normalize();
//    m_P.block(stateOffset+3,0,4,totalStates) = q_norm_To_q * m_P.block(stateOffset+3,0,4,totalStates);
//    m_P.block(0,stateOffset+3,totalStates,4) = m_P.block(0,stateOffset+3,totalStates,4) * q_norm_To_q.transpose();
    stateOffset += 7 + (*ait)->GetNumberOfPoints();
  }
#endif

  //Initialize new points ===========================================================
  double rhoInit = 1;
  double rhoSigma = 0.1;
#ifdef FANCY_P
  std::vector<Eigen::Vector2d> tnp;
  for (size_t c = 0; c < cornerStatus.size(); c++)
  {
//    for (size_t c = 0; c < newPoints.size(); c++)
    if (cornerStatus[c] == 2)
    {
      printf("a\n");
      tnp.push_back(newPoints[c]);
      cornerStatus[c] = 1;
    }
  }
  if (tnp.size() > 0)
  {
    PAnchor *pa = new PAnchor;
//      PAnchor *pa = new PAnchor(mCam, image, newPoints, rhoInit);
    pa->CreateFeatureData(mCam, image, tnp, rhoInit);
    int curNumPoints = pa->GetNumberOfPoints();

//      stateOffset += 7 + curNumPoints;  //state offset grows
    totalStates += 7 + curNumPoints;  //total states are now increasing

    //copy current camera pose into anchor
    pa->m_frwc = m_camStates.template head<3>();
    pa->m_fqwc = m_camStates.template segment<4>(3);

    //now do the Covariance bookkeeping...
    int thisStates = 7 + pa->GetNumberOfPoints();

    //the covariance WRT to camera states is a copy of the camera state covariance
    pa->m_Pxy = Eigen::Matrix<double, states, Eigen::Dynamic>::Zero(states, thisStates);
    pa->m_Pxy.template topLeftCorner<13,7>() = m_Pxx.template topLeftCorner<13,7>();
    pa->m_Pyy = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(thisStates, thisStates);
    pa->m_Pyy.template topLeftCorner<7,7>() = m_Pxx.template topLeftCorner<7,7>();
    pa->m_Pyy.bottomRightCorner(pa->GetNumberOfPoints(), pa->GetNumberOfPoints()) = Eigen::MatrixXd::Identity(pa->GetNumberOfPoints(), pa->GetNumberOfPoints()) * rhoSigma;

    //for all anchors before this one
    for (typename std::list<PAnchor *>::iterator ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
    {
      PAnchor &cur_anchor = *(*ait);
      int numStates = 7+cur_anchor.GetNumberOfPoints();
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Pyy = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(numStates, thisStates);
      Pyy.template topLeftCorner<7,7>() = cur_anchor.m_Pxy.template topLeftCorner<7,7>();
      pa->m_PyyList.push_back(Pyy);
    }

    //as last action, push the new anchor to the list
    m_Anchors.push_back(pa);
  }
#else
  //with matlab style bookkeeping, if there are no anchors, create them all now
  bool firstTime = false;
  if (m_Anchors.size() == 0)
  {
    firstTime = true;
    //TODO this is ugly we need to know number of points per anchor in advance...
    int ppa = 1;
    int curNumPoints = 0;
    PAnchor *pa = new PAnchor;
    std::vector<Eigen::Vector2d> tnp;
    for (size_t c = 0; c < cornerStatus.size(); c++)
    {
      tnp.push_back(newPoints[c]);
      curNumPoints++;
      printf("f\n");

      //if ppa is reached, fill previous anchor and start a new anchor
      if (curNumPoints == ppa || c == cornerStatus.size()-1)
      {
        //add current anchor
        pa->CreateFeatureData(mCam, image, tnp, rhoInit);
        m_Anchors.push_back(pa);
        totalStates += 7 + pa->GetNumberOfPoints();
        printf("a\n");
        //create new anchor
        pa = new PAnchor;
        tnp.clear();
        curNumPoints = 0;
      }
    }

    //now make sure that the state vector and covariance matrix is of correct size
    if (m_x.rows() < totalStates) m_x.conservativeResize(totalStates);
    if (m_P.rows() < totalStates || m_P.cols() < totalStates) m_P.conservativeResize(totalStates, totalStates);
  }
  //now go and initialize all points with status 2
  stateOffset = states;
  typename std::list<PAnchor *>::iterator ait = m_Anchors.begin();  //at the same time loop through anchors and features to save time
  typename std::list<typename PAnchor::AnchorFeature *>::iterator pit = (*ait)->m_Features.begin();
  std::vector<Eigen::Vector2d> tnp;
  bool initAnchor = false;
  //loop through all corners and keep adding them to the anchors
  for (size_t c = 0; c < cornerStatus.size(); c++)
  {
    PAnchor &cur_anchor = *(*ait);

    //as soon as one corner in the current anchor is to be initialized, the whole anchor will be reset
    if (cornerStatus[c] == 2)
    {
      initAnchor = true;
      cornerStatus[c] = 1;
    }

    tnp.push_back(newPoints[c]);

    //if the current corner was the last one belonging to this anchor, run the init
    if (initAnchor && tnp.size() == cur_anchor.GetNumberOfPoints())
    {
      cur_anchor.CreateFeatureData(mCam, image, tnp, rhoInit);
      int curNumPoints = cur_anchor.GetNumberOfPoints();

      //copy state and init rho
      m_x.segment(stateOffset + 0, 3) = m_x.template head<3>();
      m_x.segment(stateOffset + 3, 4) = m_x.template segment<4>(3);
      m_x.segment(stateOffset + 7, curNumPoints) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(curNumPoints, 1) * rhoInit;

      //fill in P matrix
//      if (!firstTime)
//      {
        m_P.block(stateOffset, 0, 7 + curNumPoints, totalStates) = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(7 + curNumPoints, totalStates);
        m_P.block(0, stateOffset, totalStates, 7 + curNumPoints) = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(totalStates, 7 + curNumPoints);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(totalStates, totalStates);
        J.block(stateOffset, 0, 7, totalStates) = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(7, totalStates);
        m_P = J * m_P * J.transpose();
//      }
//      else
//      {
//        m_P.block(stateOffset, 0, 7 + curNumPoints, totalStates) = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Ones(7 + curNumPoints, totalStates)*1000;
//        m_P.block(0, stateOffset, totalStates, 7 + curNumPoints) = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Ones(totalStates, 7 + curNumPoints)*1000;
//        m_P.block(stateOffset, stateOffset, 7, 7) = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(7,7);
//      }
        //depth variance
        m_P.block(stateOffset + 7, stateOffset + 7, curNumPoints, curNumPoints) = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(curNumPoints, curNumPoints) * rhoSigma;
    }

    //next feature
    ++pit;
    //or if last feature, go to next anchor
    if (pit == cur_anchor.m_Features.end())
    {
      ++ait; stateOffset += 7 + cur_anchor.GetNumberOfPoints();
      tnp.clear();
      initAnchor = false;
      if (ait != m_Anchors.end())
        pit = (*ait)->m_Features.begin();
    }
  }
#endif

//#ifdef FANCY_P
//  BuildStateAndCovariance(x, P);
//#endif
//  std::ofstream file;
//  file.open("P.txt");
//  file << P;
//  file.close();

  //compute final projections =================================================
#ifdef FANCY_P
  const Eigen::Vector3d &r_wc = m_camStates.template head<3>();
  const Eigen::Vector4d &q_wc = m_camStates.template segment<4>(3);
#else
  const Eigen::Vector3d &r_wc = m_x.template head<3>();
  const Eigen::Vector4d &q_wc = m_x.template segment<4>(3);
#endif
  Eigen::Matrix3d R_wc, R_cw;
  R_wc << q_wc(0)*q_wc(0)+q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(1)*q_wc(2)-q_wc(0)*q_wc(3)) , 2.*(q_wc(3)*q_wc(1)+q_wc(0)*q_wc(2)),
          2.*(q_wc(1)*q_wc(2)+q_wc(0)*q_wc(3)) , q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)+q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(2)*q_wc(3)-q_wc(0)*q_wc(1)),
          2.*(q_wc(3)*q_wc(1)-q_wc(0)*q_wc(2)) , 2.*(q_wc(2)*q_wc(3)+q_wc(0)*q_wc(1)), q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)+q_wc(3)*q_wc(3);

  R_cw = R_wc.transpose();
  m_last_projections.clear();
  stateOffset = states;
  int featureOffset = 0;
  for (typename std::list<PAnchor *>::iterator ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
  {
    PAnchor &cur_anchor = *(*ait);
#ifdef FANCY_P
    const Eigen::Vector3d &fr_wc = cur_anchor.m_frwc;
    const Eigen::Vector4d &fq_wc = cur_anchor.m_fqwc;
#else
    const Eigen::Vector3d &fr_wc = m_x.template segment<3>(stateOffset+0);
    const Eigen::Vector4d &fq_wc = m_x.template segment<4>(stateOffset+3);
#endif
    //read anchor orientation
    Eigen::Matrix3d fR_wc;
    {
      const Eigen::Vector4d &q_wc = fq_wc;
      fR_wc << q_wc(0)*q_wc(0)+q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(1)*q_wc(2)-q_wc(0)*q_wc(3)) , 2.*(q_wc(3)*q_wc(1)+q_wc(0)*q_wc(2)),
                2.*(q_wc(1)*q_wc(2)+q_wc(0)*q_wc(3)) , q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)+q_wc(2)*q_wc(2)-q_wc(3)*q_wc(3) , 2.*(q_wc(2)*q_wc(3)-q_wc(0)*q_wc(1)),
                2.*(q_wc(3)*q_wc(1)-q_wc(0)*q_wc(2)) , 2.*(q_wc(2)*q_wc(3)+q_wc(0)*q_wc(1)), q_wc(0)*q_wc(0)-q_wc(1)*q_wc(1)-q_wc(2)*q_wc(2)+q_wc(3)*q_wc(3);
    }
    int localFeatureCounter = 0;
    typename std::list<typename PAnchor::AnchorFeature *>::iterator pit;
    for (pit = cur_anchor.m_Features.begin(); pit != cur_anchor.m_Features.end(); ++pit)
    {
      typename PAnchor::AnchorFeature &af = *(*pit);
#ifdef FANCY_P
      double rho = af.m_rho(0);
#else
      double rho = m_x.template segment<1>(stateOffset+7+localFeatureCounter)(0);
#endif
      //project current point into the current camera and compute derivatives
      Eigen::Vector3d h_ci = R_cw * (rho * (fr_wc - r_wc) + fR_wc*af.m_m);
      Eigen::Vector2d h_ui;
      Eigen::Matrix<double, 2, 3> Jproj;
      mCam.spaceToPlane(h_ci, h_ui, Jproj);
      if (cornerStatus[featureOffset] == 1)
        m_last_projections.push_back(h_ui);
      else
        m_last_projections.push_back(Eigen::Vector2d::Zero());

      localFeatureCounter++;
      featureOffset++;
    }
    stateOffset+= 7 + cur_anchor.GetNumberOfPoints();
  }
}


//==================================================================================================
//   Helper Functions
//==================================================================================================


template <int patchsize, int pyrlevels, int states>
void EKFPhotometric<patchsize, pyrlevels, states>::RemovePoint()
{

}

template <int patchsize, int pyrlevels, int states>
void EKFPhotometric<patchsize, pyrlevels, states>::RemoveAnchor()
{

}

template <int patchsize, int pyrlevels, int states>
int EKFPhotometric<patchsize, pyrlevels, states>::GetTotalNumberOfStates()
{
  int totalNumberOfAnchorStates = 0;

  //first compute sizes and offsets
  typename std::list<PAnchor *>::iterator ait;
  for (ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
  {
    totalNumberOfAnchorStates += 7+(*ait)->GetNumberOfPoints();
  }

  return states + totalNumberOfAnchorStates;
}

#ifdef FANCY_P
template <int patchsize, int pyrlevels, int states>
void EKFPhotometric<patchsize, pyrlevels, states>::BuildStateAndCovariance(Eigen::Matrix<double, Eigen::Dynamic,1> &x, Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &P)
{
  int totalStates = GetTotalNumberOfStates();

  if (x.rows() != totalStates || x.cols() != 1)
    x.resize(totalStates, 1);

  if (P.rows() != totalStates || P.cols() != totalStates)
    P.resize(totalStates, totalStates);

//  P = P.Ones(totalStates, totalStates) * 10;

  x.head<states>() = m_camStates;
  P.topLeftCorner<states, states>() = m_Pxx;

  //then start filling in anchor covariances
  int x_pos = states;
  for (typename std::list<PAnchor *>::iterator ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
  {
    PAnchor &cur_anchor = *(*ait);

    x.template segment<3>(x_pos+0) = cur_anchor.m_frwc;
    x.template segment<4>(x_pos+3) = cur_anchor.m_fqwc;
    int featureOffset = 0;
    typename std::list<typename PAnchor::AnchorFeature *>::iterator pit;
    for (pit = cur_anchor.m_Features.begin(); pit != cur_anchor.m_Features.end(); ++pit)
    {
      x.segment<1>(x_pos+7+featureOffset) = (*pit)->m_rho;
      featureOffset++;
    }

    //write cam-anchor covariance into P
    const Eigen::Matrix<double, states, Eigen::Dynamic> &curPxy = cur_anchor.m_Pxy;
    P.block(0, x_pos, states, curPxy.cols()) = curPxy;
    P.block(x_pos, 0, curPxy.cols(), states) = curPxy.transpose();

    //write other anchor-anchor covariances into P
    int y_pos = states;
    typename std::list<typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >::iterator fit;
    for (fit = cur_anchor.m_PyyList.begin(); fit != cur_anchor.m_PyyList.end(); ++fit)
    {
      const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &cPyy = *fit;
      P.block(y_pos, x_pos, cPyy.rows(), cPyy.cols()) = cPyy;
      P.block(x_pos, y_pos, cPyy.cols(), cPyy.rows()) = cPyy.transpose();
      y_pos += cPyy.rows();
    }

    //write own covariance
    P.block(y_pos, x_pos, cur_anchor.m_Pyy.rows(), cur_anchor.m_Pyy.cols()) = cur_anchor.m_Pyy;
    x_pos += 7+cur_anchor.GetNumberOfPoints();
  }
}

template <int patchsize, int pyrlevels, int states>
void EKFPhotometric<patchsize, pyrlevels, states>::StoreStateAndCovariance(const Eigen::Matrix<double, Eigen::Dynamic,1> &x, const Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &P)
{
  m_camStates = x.head<states>();
  m_Pxx = P.topLeftCorner<states, states>();

  //then start filling in anchor covariances
  int x_pos = states;
  typename std::list<PAnchor *>::iterator ait;
  for (ait = m_Anchors.begin(); ait != m_Anchors.end(); ++ait)
  {
    PAnchor &cur_anchor = *(*ait);

    cur_anchor.m_frwc = x.template segment<3>(x_pos+0);
    cur_anchor.m_fqwc = x.template segment<4>(x_pos+3);
    int featureOffset = 0;
    typename std::list<typename PAnchor::AnchorFeature *>::iterator pit;
    for (pit = cur_anchor.m_Features.begin(); pit != cur_anchor.m_Features.end(); ++pit)
    {
      (*pit)->m_rho = x.segment<1>(x_pos+7+featureOffset);
      featureOffset++;
    }

    //read cam-anchor covariance from P
    cur_anchor.m_Pxy = P.block(0, x_pos, states, cur_anchor.m_Pxy.cols());

    //read other anchor-anchor covariances from P
    int y_pos = states;
    typename std::list<typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >::iterator fit;
    for (fit = cur_anchor.m_PyyList.begin(); fit != cur_anchor.m_PyyList.end(); ++fit)
    {
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &cPyy = (*fit);
      cPyy = P.block(y_pos, x_pos, cPyy.rows(), cPyy.cols());
      y_pos += cPyy.rows();
    }

    //write own covariance
    cur_anchor.m_Pyy = P.block(y_pos, x_pos, cur_anchor.m_Pyy.rows(), cur_anchor.m_Pyy.cols());
    x_pos += 7+cur_anchor.GetNumberOfPoints();
  }
}
#endif

// Find the warping matrix and search level, this is assuming that the reference patch is looking perpenticular into the reference view, this makes sense since this function is used for point initialization
template <int patchsize, int pyrlevels, int states>
int EKFPhotometric<patchsize, pyrlevels, states>::GetWarpMatrixAndLevel(double depth, const Eigen::Vector2d &ref_uv, int ref_level, const Eigen::Matrix3d &R_cw, const Eigen::Vector3d &r_wc, const Eigen::Matrix3d &fR_wc, const Eigen::Vector3d &fr_wc, Eigen::Matrix2d &A)
{
  // Project the source keyframe's one-pixel-right and one-pixel-down vectors into the current view
  Eigen::Vector3d dirCenter, dirRight, dirDown;
  double lvlscale = (1<<ref_level);
  mCam.planeToSpace(ref_uv, dirCenter);
  mCam.planeToSpace(ref_uv + Eigen::Vector2d(lvlscale, 0), dirRight);
  mCam.planeToSpace(ref_uv + Eigen::Vector2d(0, lvlscale), dirDown);
  //go to correct depth
  Eigen::Vector3d wCenter, wRight, wDown;
  dirCenter *= depth / dirCenter(2);
  dirRight *= depth / dirRight(2);
  dirDown *= depth / dirDown(2);
  //transform to world coordinates
  dirCenter = fR_wc * dirCenter + fr_wc;
  dirRight = fR_wc * dirRight + fr_wc;
  dirDown = fR_wc * dirDown + fr_wc;
  //transform to current camera
  dirCenter = R_cw * dirCenter - r_wc;
  dirRight = R_cw * dirRight - r_wc;
  dirDown = R_cw * dirDown - r_wc;
  //project into image
  Eigen::Vector2d cur_pixelPos, cur_dirRight, cur_dirDown;
  mCam.spaceToPlane(dirCenter, cur_pixelPos);
  mCam.spaceToPlane(dirRight, cur_dirRight);
  mCam.spaceToPlane(dirDown, cur_dirDown);

  // Calculate in-image derivatives of source image pixel motions:
  A.col(0) = (cur_dirRight - cur_pixelPos);
  A.col(1) = (cur_dirDown - cur_pixelPos);

  double dDet = A.determinant();
  int searchLevel = 0;

  while (dDet > 3 && searchLevel < pyrlevels-1)
  {
    searchLevel++;
    dDet *= 0.25;
  };

  if (::isnan(A(0,0)))
  {
    printf(".\n");
    return -1;
  }

  if (dDet > 3 || dDet < 0.25)
  {
    return -1;
  }
  else
    return searchLevel;
}

// This function generates the warped search template.
template <int patchsize, int pyrlevels, int states>
bool EKFPhotometric<patchsize, pyrlevels, states>::GetPredictedPatch(const Eigen::Matrix2d &A, const cv::Mat &im, const Eigen::Vector2d &ref_pixelPos, int search_level, cv::Mat &patch)
{
//  Eigen::Matrix2f A_ref_cur = A.inverse().cast<float>() * (1<<search_level);
  Eigen::Matrix2f A_ref_cur = A.cast<float>() / (1<<search_level);
  float halfpatch_size = patchsize/2+1; //extract patch with 1 pixel border for gradient
  int nOutside = 0;

  //check that patch container has enough space
  if (patch.cols != patchsize+2 || patch.rows != patchsize+2)
  {
    patch = cv::Mat(patchsize+2, patchsize+2, CV_32F);
  }

  // Perform the warp on a larger patch.
  float* patch_ptr = (float*)patch.data;
  const Eigen::Vector2f px_ref_pyr = ref_pixelPos.cast<float>() / (1<<search_level);
  for (int y=0; y<patch.rows; ++y)
  {
    for (int x=0; x<patch.cols; ++x, ++patch_ptr)
    {
      Eigen::Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
      const Eigen::Vector2f px(A_ref_cur*px_patch + px_ref_pyr);
      if (px[0]<0 || px[1]<0 || px[0]>=im.cols-1 || px[1]>=im.rows-1)
      {
        *patch_ptr = 0;
        nOutside++;
      }
      else
      {
        float x = px[0];
        float y = px[1];
        const int lx = (int) x;
        const int ly = (int) y;
        x -= lx;
        y -= ly;
        *patch_ptr =  (1 - y) * ((1 - x) * im.at<unsigned char>(ly,lx) + x * im.at<unsigned char>(ly,lx + 1)) + y * ((1 - x) * im.at<unsigned char>(ly + 1,lx) + x * im.at<unsigned char>(ly + 1,lx + 1));
      }
    }
  }

  if (nOutside)
    return false;
  else
    return true;
}

}
