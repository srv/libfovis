#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <string>
#include <vector>

#include <fovis/fovis.hpp>

#include "lodepng.h"
#include "draw.hpp"

//#define dbg(...) do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)
#define dbg(...)
#define info(...) do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)
#define err(...) do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)

struct TimestampFilename
{
  TimestampFilename(int64_t ts, const std::string& fname) :
    timestamp(ts), filename(fname)
  {}
  int64_t timestamp;
  std::string filename;
};

static std::vector<TimestampFilename> read_file_index(const std::string& indexFile)
{
  FILE* fp = fopen(indexFile.c_str(), "r");
  std::vector<TimestampFilename> result;
  if(!fp) {
    perror("fopen");
    return result;
  }
  char linebuf[1024];
  int linenum = 0;
  while(!feof(fp)) {
    if(!fgets(linebuf, sizeof(linebuf), fp))
      break;
    linenum++;
    if(strlen(linebuf) == 0)
      break;
    if(linebuf[0] == '#')
      continue;
    long seconds;
    long microseconds;
    char png_fname[1024];
    if(3 != sscanf(linebuf, "%ld.%ld %s", &seconds, &microseconds, png_fname)) {
      err("%s:%d Parse error", indexFile.c_str(), linenum);
      fclose(fp);
      return result;
    }
    int64_t timestamp = seconds * 1000000 + microseconds;
    result.push_back(TimestampFilename(timestamp, png_fname));
  }
  return result;
}

static void write_pgm(const std::string& fname, const uint8_t* pixels,
    int width, int height, int stride)
{
  FILE* fp = fopen(fname.c_str(), "wb");
  if(!fp) {
    perror("fopen");
    return;
  }
  fprintf(fp, "P5\n%d\n%d\n%d\n", width, height, 255);
  int row;
  for (row=0; row<height; row++) {
    if (1 != fwrite(pixels + row*stride, width, 1, fp)) {
      fclose(fp);
      return;
    }
  }
  fclose(fp);
  return;
}

static void usage()
{
  fprintf(stderr, "usage: fv-tum-rgbd [options] <dataset_dir> <output_dir>\n"
      "\n"
      "This program demonstrates using FOVIS with the TUM-RGBD dataset\n"
      "\n"
      "   http://vision.in.tum.de/data/datasets/rgbd-dataset\n"
      "\n"
      "OPTIONS:\n"
      "\n"
      "  --help, -h       Shows this help text\n"
      "  --camera ID      Uses the calibration parameters for camera ID.  Must be\n"
      "                   one of 'fr1', 'fr2', or 'fr3'.  This options is required\n"
      "  --visualize, -v  Visualizes the motion estimation results by drawing\n"
      "                   reference and target images, features, and feature\n"
      "                   matches.  The visualized results are saved to individual\n"
      "                   PNG files in <output_dir>/vis\n"
      "\n"
      );
  exit(1);
}

int main(int argc, char **argv)
{
  const char *optstring = "hc:v";
  int c;
  struct option long_opts[] = {
    { "help", no_argument, 0, 'h' },
    { "camera", required_argument, 0, 'c' },
    { "visualize", no_argument, 0, 'v' },
    { 0, 0, 0, 0 }
  };

  std::string camera_id;
  bool visualize = false;

  while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
      case 'c':
        camera_id = optarg;
        break;
      case 'v':
        visualize = true;
        break;
      default:
        usage();
        return 1;
    };
  }

  if(optind >= argc - 1) {
    usage();
  }

  std::string datadir = argv[optind];
  std::string outdir = argv[optind+1];

  if(camera_id.empty()) {
    err("Missing --camera parameter.  Run with -h for options.");
    return 1;
  }

  fovis::CameraIntrinsicsParameters camParams;
  camParams.width = 640;
  camParams.height = 480;

  // RGB camera parameters
  if(camera_id == "fr1") {
    camParams.fx = 517.306408;
    camParams.fy = 516.469215;
    camParams.cx = 318.643040;
    camParams.cy = 255.313989;
    camParams.k1 = 0.262383;
    camParams.k2 = -0.953104;
    camParams.p1 = -0.005358;
    camParams.p2 = 0.002628;
    camParams.k3 = 1.163314;
  } else if(camera_id == "fr2") {
    camParams.fx = 520.908620;
    camParams.fy = 521.007327;
    camParams.cx = 325.141442;
    camParams.cy = 249.701764;
    camParams.k1 = 0.231222;
    camParams.k2 = -0.784899;
    camParams.p1 = -0.003257;
    camParams.p2 = -0.000105;
    camParams.k3 =  0.917205;
  } else if(camera_id == "fr3") {
    camParams.fx = 537.960322;
    camParams.fy = 539.597659;
    camParams.cx = 319.183641;
    camParams.cy = 247.053820;
    camParams.k1 = 0.026370;
    camParams.k2 = -0.100086;
    camParams.p1 = 0.003138;
    camParams.p2 = 0.002421;
    camParams.k3 = 0.000000;
  } else {
    err("Unknown camera id [%s]", camera_id.c_str());
    return 1;
  }

  info("Loading data from: [%s]\n", datadir.c_str());

  fovis::Rectification rect(camParams);

  // If we wanted to play around with the different VO parameters, we could set
  // them here in the "options" variable.
  fovis::VisualOdometryOptions options =
      fovis::VisualOdometry::getDefaultOptions();

  // setup the visual odometry
  fovis::VisualOdometry odom(&rect, options);

  // create the output trajectory file
  std::string traj_fname = outdir + "/traj.txt";
  FILE* traj_fp = fopen(traj_fname.c_str(), "w");
  if(!traj_fp) {
    err("Unable to create %s - %s", traj_fname.c_str(), strerror(errno));
    return 1;
  }
  std::string gray_pgm_dir = outdir + "/gray";
  mkdir(gray_pgm_dir.c_str(), 0755);
  std::string depth_pgm_dir = outdir + "/depth";
  mkdir(depth_pgm_dir.c_str(), 0755);
  std::string vis_dir = outdir + "/vis";
  mkdir(vis_dir.c_str(), 0755);

  // read the RGB and depth index files
  std::vector<TimestampFilename> rgb_fnames =
    read_file_index(datadir + "/rgb.txt");
  std::vector<TimestampFilename> depth_fnames =
    read_file_index(datadir + "/depth.txt");
  int depth_fname_index = 0;

  DrawImage draw_img(camParams.width, camParams.height * 2);

  for(int rgb_fname_index=0;
      rgb_fname_index < rgb_fnames.size();
      rgb_fname_index++) {

    int64_t timestamp = rgb_fnames[rgb_fname_index].timestamp;
    std::string rgb_fname =
      datadir + "/" + rgb_fnames[rgb_fname_index].filename;
    long long ts_sec = timestamp / 1000000;
    long ts_usec = timestamp % 1000000;
    char ts_str[80];
    snprintf(ts_str, 80, "%lld.%06ld", ts_sec, ts_usec);

    // match the RGB image up with a depth image
    bool matched_depth_image = false;
    while(depth_fname_index < depth_fnames.size()) {
      int64_t depth_ts = depth_fnames[depth_fname_index].timestamp;

      // declare a depth image match if the depth image timestamp is within
      // 40ms of the RGB image.
      int64_t dt_usec = depth_ts - timestamp;
      if(dt_usec > 40000) {
        dbg("  stop %lld.%06ld (dt %f)",
            (long long)(depth_ts / 1000000),
            (long)(depth_ts % 1000000),
            dt_usec / 1000.);
        break;
      } else if(dt_usec < -40000) {
        dbg("  skip %lld.%06ld (dt %f)",
            (long long)(depth_ts / 1000000),
            (long)(depth_ts % 1000000),
            dt_usec / 1000.);
        depth_fname_index++;
      } else {
        matched_depth_image = true;
        dbg("  mtch %lld.%06ld (dt %f)",
            (long long)(depth_ts / 1000000),
            (long)(depth_ts % 1000000),
            dt_usec / 1000.);
        break;
      }
    }

    if(!matched_depth_image) {
      // didn't find a depth image with a close enough timestamp.  Skip this
      // RGB image.
      info("# skip %s", ts_str);
      continue;
    }

    std::string depth_fname =
      datadir + "/" + depth_fnames[depth_fname_index].filename;
    depth_fname_index++;

    // read RGB data
    std::vector<uint8_t> rgb_data;
    unsigned rgb_width;
    unsigned rgb_height;
    std::vector<uint8_t> rgb_png_data;
    lodepng::load_file(rgb_png_data, rgb_fname.c_str());
    if(rgb_png_data.empty()) {
      err("Failed to load %s", rgb_fname.c_str());
      continue;
    }
    if(0 != lodepng::decode(rgb_data, rgb_width, rgb_height,
        rgb_png_data, LCT_RGB, 8)) {
      err("Error decoding PNG %s", rgb_fname.c_str());
      continue;
    }

    // convert RGB data to grayscale
    std::vector<uint8_t> gray_data(rgb_width*rgb_height);
    const uint8_t* rgb_pixel = &rgb_data[0];
    for(int pixel_index=0; pixel_index<rgb_width*rgb_height; pixel_index++) {
      uint8_t r = rgb_pixel[0];
      uint8_t g = rgb_pixel[1];
      uint8_t b = rgb_pixel[2];
      gray_data[pixel_index] = (r + g + b) / 3;
      rgb_pixel += 3;
    }

    // write gray image out.
    write_pgm(outdir + "/gray/" + ts_str + "-gray.pgm",
        &gray_data[0], rgb_width, rgb_height, rgb_width);

    // read depth data
    std::vector<uint8_t> depth_data_u8;
    unsigned depth_width;
    unsigned depth_height;
    std::vector<uint8_t> depth_png_data;
    lodepng::load_file(depth_png_data, depth_fname.c_str());
    if(depth_png_data.empty()) {
      err("Failed to load %s", depth_fname.c_str());
      continue;
    }
    if(0 != lodepng::decode(depth_data_u8, depth_width, depth_height,
        depth_png_data, LCT_GREY, 16)) {
      err("Error decoding PNG %s", depth_fname.c_str());
      continue;
    }

    // convert depth data to a DepthImage object
    fovis::DepthImage depth_image(camParams, depth_width, depth_height);
    std::vector<float> depth_data(depth_width * depth_height);
    for(int i=0; i<depth_width*depth_height; i++) {
      // lodepng loads 16-bit PNGs into memory with big-endian ordering.
      // swizzle the bytes to get a uint16_t
      uint8_t high_byte = depth_data_u8[i*2];
      uint8_t low_byte = depth_data_u8[i*2+1];
      uint16_t data16 = (high_byte << 8) | low_byte;
      if(0 == data16)
        depth_data[i] = NAN;
      else
        depth_data[i] = data16 / float(5000);
    }
    depth_image.setDepthImage(&depth_data[0]);

    // debug depth image
    std::vector<uint8_t> depth_pgm_data(depth_width * depth_height);
    float depth_pgm_max_depth = 5;
    for(int i=0; i<depth_width*depth_height; i++) {
      if(isnan(depth_data[i]))
        depth_pgm_data[i] = 0;
      else {
        float d = depth_data[i];
        if(d > depth_pgm_max_depth)
          d = depth_pgm_max_depth;
        depth_pgm_data[i] = 255 * d / depth_pgm_max_depth;
      }
    }
    write_pgm(outdir + "/depth/" + ts_str + "-depth.pgm",
        &depth_pgm_data[0], depth_width, depth_height, depth_width);

    // process the frame
    odom.processFrame(&gray_data[0], &depth_image);

    fovis::MotionEstimateStatusCode status = odom.getMotionEstimateStatus();
    switch(status) {
      case fovis::INSUFFICIENT_INLIERS:
        printf("# %s - insufficient inliers\n", ts_str);
        break;
      case fovis::OPTIMIZATION_FAILURE:
        printf("# %s - optimization failed\n", ts_str);
        break;
      case fovis::REPROJECTION_ERROR:
        printf("# %s - reprojection error too high\n", ts_str);
        break;
      case fovis::NO_DATA:
        printf("# %s - no data\n", ts_str);
        break;
      case fovis::SUCCESS:
      default:
        break;
    }

    // get the integrated pose estimate.
    Eigen::Isometry3d cam_to_local = odom.getPose();
    Eigen::Vector3d t = cam_to_local.translation();
    Eigen::Quaterniond q(cam_to_local.rotation());

    printf("%lld.%06ld %f %f %f %f %f %f %f\n",
        ts_sec, ts_usec, t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    fprintf(traj_fp, "%lld.%06ld %f %f %f %f %f %f %f\n",
        ts_sec, ts_usec, t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());

    // visualization
    if(visualize) {
      DrawColor status_colors[] = {
        DrawColor(255, 255, 0),
        DrawColor(0, 255, 0),
        DrawColor(255, 0, 0),
        DrawColor(255, 0, 255),
        DrawColor(127, 127, 0)
      };

      DrawColor red(255, 0, 0);
      DrawColor green(0, 255, 0);
      DrawColor blue(0, 0, 255);

      memset(&draw_img.data[0], 0, draw_img.data.size());

      const fovis::OdometryFrame* ref_frame = odom.getReferenceFrame();
      const fovis::OdometryFrame* tgt_frame = odom.getTargetFrame();

      const fovis::PyramidLevel* ref_pyr_base = ref_frame->getLevel(0);
      const uint8_t* ref_img_data = ref_pyr_base->getGrayscaleImage();
      int ref_img_stride = ref_pyr_base->getGrayscaleImageStride();

      const fovis::PyramidLevel* tgt_pyr_base = tgt_frame->getLevel(0);
      const uint8_t* tgt_img_data = tgt_pyr_base->getGrayscaleImage();
      int tgt_img_stride = tgt_pyr_base->getGrayscaleImageStride();

      int tgt_yoff = ref_pyr_base->getHeight();

      // draw the reference frame on top
      draw_gray_img_rgb(ref_img_data, ref_pyr_base->getWidth(),
          ref_pyr_base->getHeight(), ref_img_stride, 0, 0, &draw_img);

      // draw the target frame on bottom
      draw_gray_img_rgb(tgt_img_data, tgt_pyr_base->getWidth(),
          tgt_pyr_base->getHeight(), tgt_img_stride, 0, tgt_yoff, &draw_img);

      const fovis::MotionEstimator* mestimator = odom.getMotionEstimator();

      int num_levels = ref_frame->getNumLevels();
      for(int level_index=0; level_index<num_levels; level_index++) {
        // draw reference features
        const fovis::PyramidLevel* ref_level = ref_frame->getLevel(level_index);
        int num_ref_keypoints = ref_level->getNumKeypoints();
        for(int ref_kp_ind=0; ref_kp_ind<num_ref_keypoints; ref_kp_ind++) {
          const fovis::KeypointData* ref_kp = ref_level->getKeypointData(ref_kp_ind);
          int ref_u = (int)round(ref_kp->base_uv.x());
          int ref_v = (int)round(ref_kp->base_uv.y());
          draw_box_rgb(ref_u-1, ref_v-1, ref_u+1, ref_v+1, blue, &draw_img);
        }

        // draw target features
        const fovis::PyramidLevel* tgt_level = tgt_frame->getLevel(level_index);
        int num_tgt_keypoints = tgt_level->getNumKeypoints();
        for(int tgt_kp_ind=0; tgt_kp_ind<num_tgt_keypoints; tgt_kp_ind++) {
          const fovis::KeypointData* tgt_kp = tgt_level->getKeypointData(tgt_kp_ind);
          int tgt_u = (int)round(tgt_kp->base_uv.x());
          int tgt_v = (int)round(tgt_kp->base_uv.y());
          draw_box_rgb(tgt_u-1, tgt_v-1 + tgt_yoff, tgt_u+1, tgt_v+1 + tgt_yoff,
              blue, &draw_img);
        }
      }

      const fovis::FeatureMatch* matches = mestimator->getMatches();
      int num_matches = mestimator->getNumMatches();

      // draw non-inlier matches
      for(int match_index=0; match_index<num_matches; match_index++) {
        const fovis::FeatureMatch& match = matches[match_index];
        if(match.inlier)
          continue;
        const fovis::KeypointData* ref_keypoint = match.ref_keypoint;
        const fovis::KeypointData* tgt_keypoint = match.target_keypoint;

        int ref_u = (int)round(ref_keypoint->base_uv.x());
        int ref_v = (int)round(ref_keypoint->base_uv.y());

        int tgt_u = (int)round(tgt_keypoint->base_uv.x());
        int tgt_v = (int)round(tgt_keypoint->base_uv.y());

        draw_line_rgb(ref_u, ref_v,
            tgt_u, tgt_v + tgt_yoff,
            red, &draw_img);
      }

      // draw inlier matches
      for(int match_index=0; match_index<num_matches; match_index++) {
        const fovis::FeatureMatch& match = matches[match_index];
        if(!match.inlier)
          continue;
        const fovis::KeypointData* ref_keypoint = match.ref_keypoint;
        const fovis::KeypointData* tgt_keypoint = match.target_keypoint;

        int ref_u = (int)round(ref_keypoint->base_uv.x());
        int ref_v = (int)round(ref_keypoint->base_uv.y());

        int tgt_u = (int)round(tgt_keypoint->base_uv.x());
        int tgt_v = (int)round(tgt_keypoint->base_uv.y());

        draw_line_rgb(ref_u, ref_v,
            tgt_u, tgt_v + tgt_yoff,
            green, &draw_img);
      }

      // draw a couple lines indicating the VO status
      draw_box_rgb(0, tgt_yoff - 1, draw_img.width,
          tgt_yoff + 1, status_colors[status], &draw_img);

      // save visualization
      std::string vis_fname = vis_dir + "/" + ts_str + "-vis.png";
      lodepng::encode(vis_fname, &draw_img.data[0], draw_img.width,
          draw_img.height, LCT_RGB);
    }
  }

  fclose(traj_fp);

  return 0;
}
