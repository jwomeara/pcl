/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/filters/local_min_max.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <time.h>

using namespace pcl;

// first create the test point clouds
PointCloud<PointXYZ> cloud_in;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box, Minimum)
{
  PointCloud<PointXYZ> cloud_out;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_BOX);

  // small boxes - Serial
  double start_small_serial = omp_get_wtime();
  lm.setNumberOfThreads (1);
  lm.setResolution (1.0f);
  lm.filter (cloud_out);

  // small boxes - 2 Processors
  double start_small_2t = omp_get_wtime();
  lm.setNumberOfThreads (2);
  lm.filter (cloud_out);

  // small boxes - 4 Processors
  double start_small_4t = omp_get_wtime();
  lm.setNumberOfThreads (4);
  lm.filter (cloud_out);


  // large boxes - Serial
  double start_large_serial = omp_get_wtime();
  lm.setNumberOfThreads (1);
  lm.setResolution (50.0f);
  lm.filter (cloud_out);

  // large boxes - 2 Processors
  double start_large_2t = omp_get_wtime();
  lm.setNumberOfThreads (2);
  lm.filter (cloud_out);

  // large boxes - 4 Processors
  double start_large_4t = omp_get_wtime();
  lm.setNumberOfThreads (4);
  lm.filter (cloud_out);

  double stop = omp_get_wtime();

  double time_small_serial = start_small_2t - start_small_serial;

  PCL_ALWAYS ("[Box, Minimum] Resolution: 1, Serial: [Runtime: %.3fs]\n", time_small_serial);

  double time_small_2t = start_small_4t - start_small_2t;
  double speedup_small_2t = time_small_serial / time_small_2t;
  double overhead_small_2t = 2.0 * time_small_2t - time_small_serial;
  double efficiency_small_2t = speedup_small_2t / 2.0;

  PCL_ALWAYS ("[Box, Minimum] Resolution: 1, 2 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_small_2t, speedup_small_2t, overhead_small_2t, efficiency_small_2t);

  double time_small_4t = start_large_serial - start_small_4t;
  double speedup_small_4t = time_small_serial / time_small_4t;
  double overhead_small_4t = 4.0 * time_small_4t - time_small_serial;
  double efficiency_small_4t = speedup_small_4t / 4.0;

  PCL_ALWAYS ("[Box, Minimum] Resolution: 1, 4 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_small_4t, speedup_small_4t, overhead_small_4t, efficiency_small_4t);


  double time_large_serial = start_large_2t - start_large_serial;

  PCL_ALWAYS ("[Box, Minimum] Resolution: 50, Serial: [Runtime: %.3fs]\n", time_large_serial);

  double time_large_2t = start_large_4t - start_large_2t;
  double speedup_large_2t = time_large_serial / time_large_2t;
  double overhead_large_2t = 2.0 * time_large_2t - time_large_serial;
  double efficiency_large_2t = speedup_large_2t / 2.0;

  PCL_ALWAYS ("[Box, Minimum] Resolution: 50, 2 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_large_2t, speedup_large_2t, overhead_large_2t, efficiency_large_2t);

  double time_large_4t = stop - start_large_4t;
  double speedup_large_4t = time_large_serial / time_large_4t;
  double overhead_large_4t = 4.0 * time_large_4t - time_large_serial;
  double efficiency_large_4t = speedup_large_4t / 4.0;
  
  PCL_ALWAYS ("[Box, Minimum] Resolution: 50, 4 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_large_4t, speedup_large_4t, overhead_large_4t, efficiency_large_4t);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius, Minimum)
{
  PointCloud<PointXYZ> cloud_out;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_RADIUS);

  // small - Serial
  double start_small_serial = omp_get_wtime();
  lm.setNumberOfThreads (1);
  lm.setRadius (0.5f);
  lm.filter (cloud_out);

  // small - 2 Processors
  double start_small_2t = omp_get_wtime();
  lm.setNumberOfThreads (2);
  lm.filter (cloud_out);

  // small - 4 Processors
  double start_small_4t = omp_get_wtime();
  lm.setNumberOfThreads (4);
  lm.filter (cloud_out);


  // large - Serial
  double start_large_serial = omp_get_wtime();
  lm.setNumberOfThreads (1);
  lm.setRadius (25.0f);
  lm.filter (cloud_out);

  // large - 2 Processors
  double start_large_2t = omp_get_wtime();
  lm.setNumberOfThreads (2);
  lm.filter (cloud_out);

  // large - 4 Processors
  double start_large_4t = omp_get_wtime();
  lm.setNumberOfThreads (4);
  lm.filter (cloud_out);

  double stop = omp_get_wtime();

  double time_small_serial = start_small_2t - start_small_serial;

  PCL_ALWAYS ("[Radius, Minimum] Radius: 0.5, Serial: [Runtime: %.3fs]\n", time_small_serial);

  double time_small_2t = start_small_4t - start_small_2t;
  double speedup_small_2t = time_small_serial / time_small_2t;
  double overhead_small_2t = 2.0 * time_small_2t - time_small_serial;
  double efficiency_small_2t = speedup_small_2t / 2.0;

  PCL_ALWAYS ("[Radius, Minimum] Radius: 0.5, 2 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_small_2t, speedup_small_2t, overhead_small_2t, efficiency_small_2t);

  double time_small_4t = start_large_serial - start_small_4t;
  double speedup_small_4t = time_small_serial / time_small_4t;
  double overhead_small_4t = 4.0 * time_small_4t - time_small_serial;
  double efficiency_small_4t = speedup_small_4t / 4.0;

  PCL_ALWAYS ("[Radius, Minimum] Radius: 0.5, 4 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_small_4t, speedup_small_4t, overhead_small_4t, efficiency_small_4t);


  double time_large_serial = start_large_2t - start_large_serial;

  PCL_ALWAYS ("[Radius, Minimum] Radius: 25, Serial: [Runtime: %.3fs]\n", time_large_serial);

  double time_large_2t = start_large_4t - start_large_2t;
  double speedup_large_2t = time_large_serial / time_large_2t;
  double overhead_large_2t = 2.0 * time_large_2t - time_large_serial;
  double efficiency_large_2t = speedup_large_2t / 2.0;

  PCL_ALWAYS ("[Radius, Minimum] Radius: 25, 2 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_large_2t, speedup_large_2t, overhead_large_2t, efficiency_large_2t);

  double time_large_4t = stop - start_large_4t;
  double speedup_large_4t = time_large_serial / time_large_4t;
  double overhead_large_4t = 4.0 * time_large_4t - time_large_serial;
  double efficiency_large_4t = speedup_large_4t / 4.0;
  
  PCL_ALWAYS ("[Radius, Minimum] Radius: 25, 4 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_large_4t, speedup_large_4t, overhead_large_4t, efficiency_large_4t);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN, Minimum)
{
  PointCloud<PointXYZ> cloud_out;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_KNN);

  // small - Serial
  double start_small_serial = omp_get_wtime();
  lm.setNumberOfThreads (1);
  lm.setNumNeighbors (50);
  lm.filter (cloud_out);

  // small - 2 Processors
  double start_small_2t = omp_get_wtime();
  lm.setNumberOfThreads (2);
  lm.filter (cloud_out);

  // small - 4 Processors
  double start_small_4t = omp_get_wtime();
  lm.setNumberOfThreads (4);
  lm.filter (cloud_out);


  // large - Serial
  double start_large_serial = omp_get_wtime();
  lm.setNumberOfThreads (1);
  lm.setNumNeighbors (500);
  lm.filter (cloud_out);

  // large - 2 Processors
  double start_large_2t = omp_get_wtime();
  lm.setNumberOfThreads (2);
  lm.filter (cloud_out);

  // large - 4 Processors
  double start_large_4t = omp_get_wtime();
  lm.setNumberOfThreads (4);
  lm.filter (cloud_out);

  double stop = omp_get_wtime();

  double time_small_serial = start_small_2t - start_small_serial;

  PCL_ALWAYS ("[KNN, Minimum] NN: 50, Serial: [Runtime: %.3fs]\n", time_small_serial);

  double time_small_2t = start_small_4t - start_small_2t;
  double speedup_small_2t = time_small_serial / time_small_2t;
  double overhead_small_2t = 2.0 * time_small_2t - time_small_serial;
  double efficiency_small_2t = speedup_small_2t / 2.0;

  PCL_ALWAYS ("[KNN, Minimum] NN: 50, 2 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_small_2t, speedup_small_2t, overhead_small_2t, efficiency_small_2t);

  double time_small_4t = start_large_serial - start_small_4t;
  double speedup_small_4t = time_small_serial / time_small_4t;
  double overhead_small_4t = 4.0 * time_small_4t - time_small_serial;
  double efficiency_small_4t = speedup_small_4t / 4.0;

  PCL_ALWAYS ("[KNN, Minimum] NN: 50, 4 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_small_4t, speedup_small_4t, overhead_small_4t, efficiency_small_4t);


  double time_large_serial = start_large_2t - start_large_serial;

  PCL_ALWAYS ("[KNN, Minimum] NN: 500, Serial: [Runtime: %.3fs]\n", time_large_serial);

  double time_large_2t = start_large_4t - start_large_2t;
  double speedup_large_2t = time_large_serial / time_large_2t;
  double overhead_large_2t = 2.0 * time_large_2t - time_large_serial;
  double efficiency_large_2t = speedup_large_2t / 2.0;

  PCL_ALWAYS ("[KNN, Minimum] NN: 500, 2 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_large_2t, speedup_large_2t, overhead_large_2t, efficiency_large_2t);

  double time_large_4t = stop - start_large_4t;
  double speedup_large_4t = time_large_serial / time_large_4t;
  double overhead_large_4t = 4.0 * time_large_4t - time_large_serial;
  double efficiency_large_4t = speedup_large_4t / 4.0;
  
  PCL_ALWAYS ("[KNN, Minimum] NN: 500, 4 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_large_4t, speedup_large_4t, overhead_large_4t, efficiency_large_4t);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Minimum)
{
  PointCloud<PointXYZ> cloud_out;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_GRID);

  // small boxes - Serial
  double start_small_serial = omp_get_wtime();
  lm.setNumberOfThreads (1);
  lm.setResolution (1.0f);
  lm.filter (cloud_out);

  // small boxes - 2 Processors
  double start_small_2t = omp_get_wtime();
  lm.setNumberOfThreads (2);
  lm.filter (cloud_out);

  // small boxes - 4 Processors
  double start_small_4t = omp_get_wtime();
  lm.setNumberOfThreads (4);
  lm.filter (cloud_out);


  // large boxes - Serial
  double start_large_serial = omp_get_wtime();
  lm.setNumberOfThreads (1);
  lm.setResolution (50.0f);
  lm.filter (cloud_out);

  // large boxes - 2 Processors
  double start_large_2t = omp_get_wtime();
  lm.setNumberOfThreads (2);
  lm.filter (cloud_out);

  // large boxes - 4 Processors
  double start_large_4t = omp_get_wtime();
  lm.setNumberOfThreads (4);
  lm.filter (cloud_out);

  double stop = omp_get_wtime();

  double time_small_serial = start_small_2t - start_small_serial;

  PCL_ALWAYS ("[Grid, Minimum] Resolution: 1, Serial: [Runtime: %.3fs]\n", time_small_serial);

  double time_small_2t = start_small_4t - start_small_2t;
  double speedup_small_2t = time_small_serial / time_small_2t;
  double overhead_small_2t = 2.0 * time_small_2t - time_small_serial;
  double efficiency_small_2t = speedup_small_2t / 2.0;

  PCL_ALWAYS ("[Grid, Minimum] Resolution: 1, 2 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_small_2t, speedup_small_2t, overhead_small_2t, efficiency_small_2t);

  double time_small_4t = start_large_serial - start_small_4t;
  double speedup_small_4t = time_small_serial / time_small_4t;
  double overhead_small_4t = 4.0 * time_small_4t - time_small_serial;
  double efficiency_small_4t = speedup_small_4t / 4.0;

  PCL_ALWAYS ("[Grid, Minimum] Resolution: 1, 4 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_small_4t, speedup_small_4t, overhead_small_4t, efficiency_small_4t);


  double time_large_serial = start_large_2t - start_large_serial;

  PCL_ALWAYS ("[Grid, Minimum] Resolution: 50, Serial: [Runtime: %.3fs]\n", time_large_serial);

  double time_large_2t = start_large_4t - start_large_2t;
  double speedup_large_2t = time_large_serial / time_large_2t;
  double overhead_large_2t = 2.0 * time_large_2t - time_large_serial;
  double efficiency_large_2t = speedup_large_2t / 2.0;

  PCL_ALWAYS ("[Grid, Minimum] Resolution: 50, 2 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_large_2t, speedup_large_2t, overhead_large_2t, efficiency_large_2t);

  double time_large_4t = stop - start_large_4t;
  double speedup_large_4t = time_large_serial / time_large_4t;
  double overhead_large_4t = 4.0 * time_large_4t - time_large_serial;
  double efficiency_large_4t = speedup_large_4t / 4.0;
  
  PCL_ALWAYS ("[Grid, Minimum] Resolution: 50, 4 Threads: [Runtime: %.3fs, Speedup: %.3f, Overhead: %.3fs, Efficiency: %.3f]\n", time_large_4t, speedup_large_4t, overhead_large_4t, efficiency_large_4t);
}

/* ---[ */
int
main (int argc, char** argv)
{
  int low = -100;
  int high = 100;

  cloud_in.height = 1;
  cloud_in.width = 100000;
  cloud_in.is_dense = true;
  cloud_in.resize (100000);

  for (int i = 0; i < cloud_in.size (); i++)
  {
    cloud_in[i].x = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
    cloud_in[i].y = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
    cloud_in[i].z = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

