// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvs_calibration/board_detection.h"

namespace dvs_calibration {

std::vector<cv::Point2f> BoardDetection::findPattern(std::list<PointWithWeight> points, int dots_w, int dots_h, int minimum_mass)
{
  cv::Size patternsize(dots_w, dots_h); //number of centers

  std::vector<std::list<PointWithWeight> > clusters;

  // build clusters
  std::list<PointWithWeight>::iterator point_it, cluster_it;
  while (!points.empty())
  {
    std::list<PointWithWeight> current_cluster;
    point_it = points.begin();
    current_cluster.push_back(*point_it);
    point_it = points.erase(point_it);

    bool found_a_neighbor = true;
    while (found_a_neighbor)
    {
      found_a_neighbor = false;
      for (point_it = points.begin(); point_it != points.end(); ++point_it)
      {
        for (cluster_it = current_cluster.begin(); cluster_it != current_cluster.end(); ++cluster_it)
        {
          cv::Point dist = point_it->point - cluster_it->point;
          const int max_dist = 1;
          if (dist.x >= -max_dist && dist.x <= max_dist && dist.y >= -max_dist && dist.y <= max_dist)
          {
            current_cluster.push_back(*point_it);
            point_it = points.erase(point_it);
            found_a_neighbor = true;
            break;
          }
        }
      }
    }

    int cluster_mass = 0;
    for (cluster_it = current_cluster.begin(); cluster_it != current_cluster.end(); ++cluster_it)
      cluster_mass += cluster_it->weight;
    if (cluster_mass >= minimum_mass)
      clusters.push_back(current_cluster);
  }

  // find cluster centers and grid
  std::vector<cv::Point2f> centers_good;
  if (clusters.size() == dots_w * dots_h)
  {
    std::vector<cv::Point2f> centers;
    for (int i = 0; i < clusters.size(); ++i)
    {
      cv::Point2f center(0, 0);
      int mass = 0;
      std::list<PointWithWeight>::iterator cluster_it;
      for (cluster_it = clusters[i].begin(); cluster_it != clusters[i].end(); ++cluster_it)
      {
        center.x += cluster_it->point.x * cluster_it->weight;
        center.y += cluster_it->point.y * cluster_it->weight;
        mass += cluster_it->weight;
      }
      center.x /= (double)mass;
      center.y /= (double)mass;
      centers.push_back(center);
    }


    // check centers (at least 3 pixels apart)
    bool checks_passed = true;
    for (int i=0; i<centers.size(); i++)
    {
      for (int j=i+1; j<centers.size(); j++)
      {
        if (cv::norm(centers[i] - centers[j]) < 3.0)
        {
          checks_passed = false;
        }
      }
    }

    if (checks_passed)
    {
      CirclesGridClusterFinder grid(false);
      grid.findGrid(centers, patternsize, centers_good);
    }
  }

  return centers_good;
}

} // namespace
