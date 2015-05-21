/*
 * Standard pinhole camera model class
 *
 * Author: Petri Tanskanen <tpetri@inf.ethz.ch>
 */

#ifndef CAMERA_PINHOLE_H_
#define CAMERA_PINHOLE_H_

#include "CameraModel.h"
#include <Eigen/Core>
#include <vector>

namespace sfm
{

template<typename Scalar = double>
class CameraPinhole : public CameraModel<Scalar>
{
public:
    class Parameters : public CameraModel<Scalar>::Parameters
    {
    public:
        Parameters(): CameraModel<Scalar>::Parameters(CameraModelBase::PINHOLE) {}
        Parameters(const std::string& cameraName, int w, int h, const Eigen::Matrix<Scalar, 2, 1> &focal, const Eigen::Matrix<Scalar, 2, 1> &center, const std::vector<Scalar> &dist)
        : CameraModel<Scalar>::Parameters(CameraModelBase::PINHOLE, cameraName, w, h)
        , mdCenter(center)
        , mdFocal(focal)
        , mdFocalInv(1/focal(0), 1/focal(1))
        , mdDistortionParam(dist)
        {
        	CameraModel<Scalar>::Parameters::K = Eigen::Matrix<Scalar, 3, 3>::Identity();
        	CameraModel<Scalar>::Parameters::K(0,0) = focal(0); CameraModel<Scalar>::Parameters::K(1,1) = focal(1);
        	CameraModel<Scalar>::Parameters::K(0,2) = center(0); CameraModel<Scalar>::Parameters::K(1,2) = center(1);
        }

        Eigen::Matrix<Scalar, 2, 1> &center() { return mdCenter; }
        Eigen::Matrix<Scalar, 2, 1> &focal() { return mdFocal; }
        Eigen::Matrix<Scalar, 2, 1> &focalInv() { return mdFocalInv; }
        std::vector<Scalar> &distortion() { return mdDistortionParam; }

        const Eigen::Matrix<Scalar, 2, 1> &center() const { return mdCenter; }
        const Eigen::Matrix<Scalar, 2, 1> &focal() const { return mdFocal; }
        const Eigen::Matrix<Scalar, 2, 1> &focalInv() const { return mdFocalInv; }
        const std::vector<Scalar> &distortion() const { return mdDistortionParam; }

        virtual bool read(const std::string& filename) { return false; }
        virtual void write(const std::string& filename) const {}

    protected:
        Eigen::Matrix<Scalar, 2, 1> mdCenter;
        Eigen::Matrix<Scalar, 2, 1> mdFocal;
        Eigen::Matrix<Scalar, 2, 1> mdFocalInv;
    	std::vector<Scalar> mdDistortionParam;
    };

    CameraPinhole<Scalar>(Parameters &params) : m_parameters(params) { CameraModel<Scalar>::initViewRect(); }

    virtual CameraModelBase::ModelType modelType(void) const;
	virtual const std::string& cameraName(void) const;
	virtual int imageWidth(void) const;
	virtual int imageHeight(void) const;

    // Lift points from the image plane to the projective space
    virtual void planeToSpace(const Eigen::Matrix<Scalar, 2, 1>& p, Eigen::Matrix<Scalar, 3, 1>& P) const
    //%output P
    {
    	P = (p - m_parameters.center()).cwiseProduct(m_parameters.focalInv()).homogeneous();
    }

    // Projects 3D points to the image plane
    virtual void spaceToPlane(const Eigen::Matrix<Scalar, 3, 1>& P, Eigen::Matrix<Scalar, 2, 1>& p) const
    //%output p
    {
    	if (P(2) == 0) return;
		Scalar im3inv = 1 / P(2);
		p = (P.template head<2>()*im3inv).cwiseProduct(m_parameters.focal()) + m_parameters.center();
    }

    // Projects 3D points to the image plane
    // and calculates jacobian
    virtual void spaceToPlane(const Eigen::Matrix<Scalar, 3, 1>& P, Eigen::Matrix<Scalar, 2, 1>& p, Eigen::Matrix<Scalar,2,3>& J) const
    //%output p
    //%output J
    {
    	if (P(2) == 0) return;
		Scalar im3inv = 1 / P(2);
		p = (P.template head<2>()*im3inv).cwiseProduct(m_parameters.focal()) + m_parameters.center();
		J(0,0) = m_parameters.focal()(0)*im3inv; J(0,1) = 0;                              J(0,2) = -m_parameters.focal()(0)*P(0)*im3inv*im3inv;
		J(1,0) = 0;                              J(1,1) = m_parameters.focal()(1)*im3inv; J(1,2) = -m_parameters.focal()(1)*P(1)*im3inv*im3inv;
    }

    virtual const Eigen::Matrix<Scalar, 3, 3> &getUndistIntrinsic() const { return m_parameters.intrinsicMatrix(); }

    virtual void undistToPlane(const Eigen::Matrix<Scalar, 2, 1>& p_u, Eigen::Matrix<Scalar, 2, 1>& p) const
    //%output p
    {
    	p = p_u;
    }

protected:
	Parameters m_parameters;
};

template <typename Scalar>
CameraModelBase::ModelType CameraPinhole<Scalar>::modelType(void) const { return m_parameters.modelType(); }

template <typename Scalar>
const std::string& CameraPinhole<Scalar>::cameraName(void) const { return m_parameters.cameraName(); }

template <typename Scalar>
int CameraPinhole<Scalar>::imageWidth(void) const { return m_parameters.imageWidth(); }

template <typename Scalar>
int CameraPinhole<Scalar>::imageHeight(void) const { return m_parameters.imageHeight(); }

}

#endif /* CAMERA_PINHOLE_H_ */
