#ifndef COMMON_H
#define COMMON_H

// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include "handrotation.h"
#include "lossfunc.h"

#include <iostream>

using namespace Eigen;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;



inline void rgbTohsv(int r, int g, int b, float &hr, float &sr, float &vr)
{
  float rd, gd, bd, h, s, v, max, min, del, rc, gc, bc;

  /* convert RGB to HSV */
  rd = (float)r / 255.0;            /* rd,gd,bd range 0-1 instead of 0-255 */
  gd = (float)g / 255.0;
  bd = (float)b / 255.0;

  /* compute maximum of rd,gd,bd */
  if (rd>=gd) { if (rd>=bd) max = rd;  else max = bd; }
         else { if (gd>=bd) max = gd;  else max = bd; }

  /* compute minimum of rd,gd,bd */
  if (rd<=gd) { if (rd<=bd) min = rd;  else min = bd; }
         else { if (gd<=bd) min = gd;  else min = bd; }

  del = max - min;
  v = max;
  if (max != 0.0) s = (del) / max;
             else s = 0.0;

  if (s != 0.0) {
    rc = (max - rd) / del;
    gc = (max - gd) / del;
    bc = (max - bd) / del;

    if      (rd==max) h = bc - gc;
    else if (gd==max) h = 2 + rc - bc;
    else if (bd==max) h = 4 + gc - rc;

    h = h * 60;
    if (h<0) h += 360;
  }
  h /=360.f;

  hr = h;  sr = s;  vr = v;
}
#endif // COMMON_H
