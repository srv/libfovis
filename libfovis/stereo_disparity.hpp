#ifndef __fovis_stereo_disparity_hpp__
#define __fovis_stereo_disparity_hpp__

#include <inttypes.h>

#include "stereo_calibration.hpp"
#include "depth_source.hpp"
#include "frame.hpp"

namespace fovis
{

/**
 * \ingroup DepthSources
 * \brief Depth source for a calibrated stereo camera pair where a dense disparity map has been precomputed.
 *
 * Supports VO where input is left and disparity. No right
 * image required. Disparity values are floats with no
 * disparity values given by disparity==0
 * Similar in concept to DepthImage and PrimesenseDepth
 */
class StereoDisparity : public DepthSource
{
  public:
    StereoDisparity(const StereoCalibration* calib);

    ~StereoDisparity();

    /**
     * Sets the disparity map.  An internal copy of the disparity data is made,
     * so the memory at \param disparity_data is not used after this method
     * returns.
     */
    void setDisparityData(const float* disparity_data);

    bool haveXyz(int u, int v);

    void getXyz(OdometryFrame* frame);

    void refineXyz(FeatureMatch* matches,
        int num_matches,
        OdometryFrame* frame);

    double getBaseline() const { return _calib->getBaseline(); }

  private:
    typedef std::vector<std::pair<double, double> > Points2d;

    /**
     * \ingroup DepthSources
     * \brief Interpolate feature position of a non-unit uvd using the 4 neighbouring pixels
     *
     * Mirrors the interpolation method in depth_image.cpp
     */
    bool getXyzInterp(KeypointData* kpdata);

    /**
     * \ingroup DepthSources
     * \brief Given a uvd, determine the xyz reprojection position
     *
     * Equivalent to cv::reprojectImageTo3D()
     */
    Eigen::Vector3d getXyzValues(int u, int v, float disparity);

    const StereoCalibration* _calib;

    int _width;
    int _height;

    Eigen::Matrix4d *_uvd1_to_xyz;

    float* _disparity_data;
};

}

#endif
