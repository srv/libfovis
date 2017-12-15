#ifndef __fovis_stereo_calibration_hpp__
#define __fovis_stereo_calibration_hpp__

#include <inttypes.h>

#include "camera_intrinsics.hpp"
#include "rectification.hpp"

namespace fovis
{

/**
 * \ingroup DepthSources
 * \brief Calibration data structure for stereo cameras.
 */
struct StereoCalibrationParameters
{
  /**
   * Translation vector: [ x, y, z ]
   */
  double right_to_left_translation[3];
  /**
   * Rotation quaternion: [ w, x, y, z ]
   */
  double right_to_left_rotation[4];

  /**
   * Intrinsics of the left camera.
   */
  CameraIntrinsicsParameters left_parameters;

  /**
   * Intrinsics of the right camera.
   */
  CameraIntrinsicsParameters right_parameters;
};

/**
 * \ingroup DepthSources
 * \brief Computes useful information from a StereoCalibrationParameters object
 */
class StereoCalibration
{
  public:
    StereoCalibration(const StereoCalibrationParameters& params);
    ~StereoCalibration();

    /**
     * Compute the 4x4 transformation matrix mapping [ u, v, disparity, 1 ]
     * coordinates to [ x, y, z, w ] homogeneous coordinates in camera
     * space.
     */
    Eigen::Matrix4d getUvdToXyz() const {
      double fx_inv = 1./_rectified_parameters.fx;
      double base_inv = 1./getBaseline();
      double cx = _rectified_parameters.cx;
      double cy = _rectified_parameters.cy;
      Eigen::Matrix4d result;
      result <<
      fx_inv    , 0      , 0               , -cx * fx_inv ,
      0         , fx_inv , 0               , -cy * fx_inv ,
      0         , 0      , 0               , 1            ,
      0         , 0      , fx_inv*base_inv , 0;
      return result;
    }

    /**
     * \return the width of the rectified camera.
     */
    int getWidth() const {
      return _rectified_parameters.width;
    }

    /**
     * \return the height of the rectified camera.
     */
    int getHeight() const {
      return _rectified_parameters.height;
    }

    double getBaseline() const {
      return -_parameters.right_to_left_translation[0];
    }

    const Rectification* getLeftRectification() const {
      return _left_rectification;
    }

    const Rectification* getRightRectification() const {
      return _right_rectification;
    }

    const CameraIntrinsicsParameters& getRectifiedParameters() const {
      return _rectified_parameters;
    }

    /**
     * \return a newly allocated copy of this calibration object.
     */
    StereoCalibration* makeCopy() const;

  private:
    StereoCalibration() { }
    void initialize();

    StereoCalibrationParameters _parameters;
    CameraIntrinsicsParameters _rectified_parameters;
    Rectification* _left_rectification;
    Rectification* _right_rectification;
};

}

#endif
