/*
 * Base camera model class - design is held similar to camodocal
 *
 * Author: Petri Tanskanen <tpetri@inf.ethz.ch>
 */

#ifndef CAMERA_MODEL_H_
#define CAMERA_MODEL_H_

#include <Eigen/Core>
#include <vector>

namespace sfm
{

class CameraModelBase
{
public:
    typedef enum ModelType_
    {
    	PINHOLE,
        ATAN,
        SCARAMUZZA
    } ModelType;
};


template<typename Scalar = double>
class CameraModel : public CameraModelBase
{
public:
    class Parameters
    {
    public:
        Parameters(ModelType modelType) : m_modelType(modelType), m_imageWidth(0), m_imageHeight(0) {}
        Parameters(ModelType modelType, const std::string& cameraName, int w, int h) : m_modelType(modelType), m_cameraName(cameraName), m_imageWidth(w), m_imageHeight(h) {}

        ModelType& modelType(void) { return m_modelType; }
        std::string& cameraName(void) { return m_cameraName; }
        int& imageWidth(void) { return m_imageWidth; }
        int& imageHeight(void) { return m_imageHeight; }

        ModelType modelType(void) const { return m_modelType; }
        const std::string& cameraName(void) const { return m_cameraName; }
        int imageWidth(void) const { return m_imageWidth; }
        int imageHeight(void) const { return m_imageHeight; }

        virtual const Eigen::Matrix<Scalar, 3, 3> &intrinsicMatrix() const { return K; }

        virtual bool read(const std::string& filename) { return false; }
        virtual void write(const std::string& filename) const { }

    protected:
        ModelType m_modelType;
        std::string m_cameraName;
        int m_imageWidth;
        int m_imageHeight;
        Eigen::Matrix<Scalar, 3, 3> K;
    };

    virtual ModelType modelType(void) const = 0;
    virtual const std::string& cameraName(void) const = 0;
    virtual int imageWidth(void) const = 0;
    virtual int imageHeight(void) const = 0;

    bool insideImage(Eigen::Matrix<Scalar, 2, 1>&p) const { return !(p(0) < 0 || p(1) < 0 || p(0) > imageWidth() || p(1) > imageHeight()); }

    void GetViewRect(Eigen::Matrix<Scalar, 2, 1> &topLeft, Eigen::Matrix<Scalar, 2, 1> &bottomRight) const { topLeft = mTopLeft; bottomRight = mBottomRight; }
    void CreateCameraFrustum(Scalar near, Scalar far, float *result) const
	{
		float left = mTopLeft(0) * near;
		float right = mBottomRight(0) * near;
		float top = mTopLeft(1) * near;
		float bottom = mBottomRight(1) * near;

	    result[0] = 2.f * near / (right - left);
	    result[1] = 0.f;
	    result[2] = 0.f;
	    result[3] = 0.f;

	    result[4] = 0.f;
	    result[5] = 2.f * near / (top - bottom);
	    result[6] = 0.f;
	    result[7] = 0.f;

	    result[8] = -(right + left) / (right - left);
	    result[9] = -(top + bottom) / (top - bottom);
	    result[10] = -(far + near) / (near - far);
	    result[11] = 1.f;

	    result[12] = 0.f;
	    result[13] = 0.f;
	    result[14] = (2.f * far * near) / (near - far);
	    result[15] = 0.f;
	}

    // Lift points from the image plane to the projective space
    virtual void planeToSpace(const Eigen::Matrix<Scalar, 2, 1>& p, Eigen::Matrix<Scalar, 3, 1>& P) const = 0;
    //%output P

    // Projects 3D points to the image plane
    virtual void spaceToPlane(const Eigen::Matrix<Scalar, 3, 1>& P, Eigen::Matrix<Scalar, 2, 1>& p) const = 0;
    //%output p

    // Projects 3D points to the image plane
    // and calculates jacobian
    virtual void spaceToPlane(const Eigen::Matrix<Scalar, 3, 1>& P, Eigen::Matrix<Scalar, 2, 1>& p, Eigen::Matrix<Scalar,2,3>& J) const = 0;
    //%output p
    //%output J

    virtual const Eigen::Matrix<Scalar, 3, 3> &getUndistIntrinsic() const = 0;

    virtual void undistToPlane(const Eigen::Matrix<Scalar, 2, 1>& p_u, Eigen::Matrix<Scalar, 2, 1>& p) const = 0;
    //%output p

protected:
    //this function is called from the subclasses after their params are loaded
	void initViewRect()
	{
		std::vector < Eigen::Vector2d > v;
		Eigen::Vector3d tmp;
		planeToSpace(Eigen::Vector2d(0, 0), tmp);
		v.push_back(tmp.head<2>() / tmp(2));
		planeToSpace(Eigen::Vector2d(imageWidth(), 0), tmp);
		v.push_back(tmp.head<2>() / tmp(2));
		planeToSpace(Eigen::Vector2d(imageWidth(), imageHeight()), tmp);
		v.push_back(tmp.head<2>() / tmp(2));
		planeToSpace(Eigen::Vector2d(0, imageHeight()), tmp);
		v.push_back(tmp.head<2>() / tmp(2));
		Eigen::Vector2d vMin = v[0];
		Eigen::Vector2d vMax = v[0];
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 2; j++)
			{
				if (v[i](j) < vMin(j))
					vMin(j) = v[i](j);
				if (v[i](j) > vMax(j))
					vMax(j) = v[i](j);
			}
		mTopLeft = vMin;
		mBottomRight = vMax;
	}

protected:
    Eigen::Matrix<Scalar, 2, 1> mTopLeft, mBottomRight;
};

}

#endif /* CAMERA_MODEL_H_ */
