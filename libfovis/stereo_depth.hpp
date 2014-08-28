#ifndef __fovis_stereo_depth_hpp__
#define __fovis_stereo_depth_hpp__

#include <inttypes.h>

#include "stereo_calibration.hpp"
#include "depth_source.hpp"
#include "frame.hpp"
#include "stereo_frame.hpp"
#include "feature_match.hpp"
#include "options.hpp"

#include "motion_estimation.hpp"

namespace fovis
{

/**
 * \ingroup DepthSources
 * \brief Stores image data for a stereo camera pair.
 *
 * TODO
 */
class StereoDepth : public DepthSource
{
  public:
    StereoDepth(const StereoCalibration* calib,
                const VisualOdometryOptions& options);

    ~StereoDepth();

    void setRightImage(const uint8_t * img);

    virtual bool haveXyz(int u, int v);

    virtual void getXyz(OdometryFrame * frame);

    virtual void refineXyz(FeatureMatch * matches,
                           int num_matches,
                           OdometryFrame * frame);

    virtual double getBaseline() const { return _calib->getBaseline(); }

  private:
    typedef std::vector<std::pair<double, double> > Points2d;

    void leftRightMatch(PyramidLevel* left_level,
                        PyramidLevel* right_level,
                        Points2d* matched_right_keypoints);

    const StereoCalibration* _calib;

    int _width;
    int _height;

    int _feature_window_size;

    int _num_pyramid_levels;

    // params for adaptive feature detector threshold
    int _fast_threshold;
    int _fast_threshold_min;
    int _fast_threshold_max;
    int _target_pixels_per_feature;
    float _fast_threshold_adaptive_gain;

    bool _use_adaptive_threshold;
    bool _require_mutual_match;
    double _max_dist_epipolar_line;
    double _max_refinement_displacement;

    StereoFrame* _right_frame;

    FeatureMatcher _matcher;
    std::vector<Points2d> _matched_right_keypoints_per_level;
    std::vector<std::vector<int> > _legal_matches;

    Eigen::Matrix4d *_uvd1_to_xyz;

    int _max_disparity;

    const VisualOdometryOptions _options;

    // matches buffer.
    FeatureMatch* _matches;
    int _num_matches;
    int _matches_capacity;

};

}

#endif
