#include "stereo_calibration.hpp"

#include <cstdio>
#include <iostream>

#include <emmintrin.h>

#include "stereo_rectify.hpp"

namespace fovis
{

StereoCalibration::StereoCalibration(const StereoCalibrationParameters& params) :
    _parameters(params)
{
  initialize();
}

StereoCalibration::~StereoCalibration()
{
  delete _left_rectification;
  delete _right_rectification;
}

void
StereoCalibration::initialize()
{
  Eigen::Quaterniond rotation_quat(_parameters.right_to_left_rotation[0],
                                   _parameters.right_to_left_rotation[1],
                                   _parameters.right_to_left_rotation[2],
                                   _parameters.right_to_left_rotation[3]);
  Eigen::Vector3d translation(_parameters.right_to_left_translation[0],
                              _parameters.right_to_left_translation[1],
                              _parameters.right_to_left_translation[2]);

  Eigen::Matrix3d left_rotation, right_rotation;
  stereo_rectify(_parameters.left_parameters, _parameters.right_parameters,
                 rotation_quat,
                 translation,
                 &left_rotation, &right_rotation, &_rectified_parameters);

  _left_rectification = new Rectification(_parameters.left_parameters,
                                          left_rotation,
                                          _rectified_parameters);

  _right_rectification = new Rectification(_parameters.right_parameters,
                                           right_rotation,
                                           _rectified_parameters);
}

StereoCalibration* StereoCalibration::makeCopy() const {
  StereoCalibration* sc = new StereoCalibration();
  sc->_parameters = _parameters;
  sc->_rectified_parameters = _rectified_parameters;
  sc->_left_rectification = _left_rectification->makeCopy();
  sc->_right_rectification = _right_rectification->makeCopy();
  return sc;
}


} /*  */
