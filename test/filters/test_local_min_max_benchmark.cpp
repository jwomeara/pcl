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
PointCloud<PointXYZ> cloud_in_10K;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box, Minimum_Negative)
{
  PointCloud<PointXYZ> cloud_out;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in_10K.makeShared ());
  lm.setStatType (lm.ST_MIN);
  lm.setNegative (true);
  lm.setLocalityType (lm.LT_BOX);

  LocalMinMax<PointXYZ> lm2;
  lm2.setInputCloud (cloud_in_10K.makeShared ());
  lm2.setStatType (lm2.ST_MIN);
  lm2.setNegative (true);
  lm2.setLocalityType (lm2.LT_BOX);

  LocalMinMax<PointXYZ> lm3;
  lm3.setInputCloud (cloud_in_10K.makeShared ());
  lm3.setStatType (lm3.ST_MIN);
  lm3.setNegative (true);
  lm3.setLocalityType (lm3.LT_BOX);

  LocalMinMax<PointXYZ> lm4;
  lm4.setInputCloud (cloud_in_10K.makeShared ());
  lm4.setStatType (lm4.ST_MIN);
  lm4.setNegative (true);
  lm4.setLocalityType (lm4.LT_BOX);

  LocalMinMax<PointXYZ> lm5;
  lm5.setInputCloud (cloud_in_10K.makeShared ());
  lm5.setStatType (lm5.ST_MIN);
  lm5.setNegative (true);
  lm5.setLocalityType (lm5.LT_BOX);

  LocalMinMax<PointXYZ> lm6;
  lm6.setInputCloud (cloud_in_10K.makeShared ());
  lm6.setStatType (lm6.ST_MIN);
  lm6.setNegative (true);
  lm6.setLocalityType (lm6.LT_BOX);

  PCL_ALWAYS ("[Box, Minimum_Negative] Small, Serial...\n");

  // small boxes - 10K - Serial
  double start_small_serial = (double)time (NULL);
  lm.setNumberOfThreads (1);
  lm.setResolution (1.0f);
  lm.filter (cloud_out);

  PCL_ALWAYS ("[Box, Minimum_Negative] Small, 2 Threads...\n");

  // small boxes - 10K - 2 Processors
  double start_small_2t = (double)time (NULL);
  lm2.setNumberOfThreads (2);
  lm2.setResolution (1.0f);
  lm2.filter (cloud_out);

  PCL_ALWAYS ("[Box, Minimum_Negative] Small, 4 Threads...\n");

  // small boxes - 10K - 4 Processors
  double start_small_4t = (double)time (NULL);
  lm3.setNumberOfThreads (4);
  lm3.setResolution (1.0f);
  lm3.filter (cloud_out);


  PCL_ALWAYS ("[Box, Minimum_Negative] Large, Serial...\n");

  // large boxes - 10K - Serial
  double start_large_serial = (double)time (NULL);
  lm4.setNumberOfThreads (1);
  lm4.setResolution (50.0f);
  lm4.filter (cloud_out);

  PCL_ALWAYS ("[Box, Minimum_Negative] Large, 2 Threads...\n");

  // large boxes - 10K - 2 Processors
  double start_large_2t = (double)time (NULL);
  lm5.setNumberOfThreads (2);
  lm5.setResolution (50.0f);
  lm5.filter (cloud_out);

  PCL_ALWAYS ("[Box, Minimum_Negative] Large, 4 Threads...\n");

  // large boxes - 10K - 4 Processors
  double start_large_4t = (double)time (NULL);
  lm6.setNumberOfThreads (4);
  lm6.setResolution (50.0f);
  lm6.filter (cloud_out);

  double stop = (double)time (NULL);

  double time_small_serial = start_small_2t - start_small_serial;

  PCL_ALWAYS ("[Box, Minimum_Negative] 10K Samples, Res: 1, Serial: [Runtime: %.3f]\n", time_small_serial);

  double time_small_2t = start_small_4t - start_small_2t;
  double speedup_small_2t = time_small_serial / time_small_2t;
  double overhead_small_2t = 2.0 * time_small_2t - time_small_serial;
  double efficiency_small_2t = speedup_small_2t / 2.0;

  PCL_ALWAYS ("[Box, Minimum_Negative] 10K Samples, Res: 1, 2 Threads: [Runtime: %.3f, Speedup: %.3f, Overhead: %.3f, Efficiency: %.3f]\n", time_small_2t, speedup_small_2t, overhead_small_2t, efficiency_small_2t);

  double time_small_4t = start_large_serial - start_small_4t;
  double speedup_small_4t = time_small_serial / time_small_4t;
  double overhead_small_4t = 4.0 * time_small_4t - time_small_serial;
  double efficiency_small_4t = speedup_small_4t / 4.0;

  PCL_ALWAYS ("[Box, Minimum_Negative] 10K Samples, Res: 1, 4 Threads: [Runtime: %.3f, Speedup: %.3f, Overhead: %.3f, Efficiency: %.3f]\n", time_small_4t, speedup_small_4t, overhead_small_4t, efficiency_small_4t);


  double time_large_serial = start_large_2t - start_large_serial;

  PCL_ALWAYS ("[Box, Minimum_Negative] 10K Samples, Res: 50, Serial: [Runtime: %.3f]\n", time_large_serial);

  double time_large_2t = start_large_4t - start_large_2t;
  double speedup_large_2t = time_large_serial / time_large_2t;
  double overhead_large_2t = 2.0 * time_large_2t - time_large_serial;
  double efficiency_large_2t = speedup_large_2t / 2.0;

  PCL_ALWAYS ("[Box, Minimum_Negative] 10K Samples, Res: 50, 2 Threads: [Runtime: %.3f, Speedup: %.3f, Overhead: %.3f, Efficiency: %.3f]\n", time_large_2t, speedup_large_2t, overhead_large_2t, efficiency_large_2t);

  double time_large_4t = stop - start_large_4t;
  double speedup_large_4t = time_large_serial / time_large_4t;
  double overhead_large_4t = 4.0 * time_large_4t - time_large_serial;
  double efficiency_large_4t = speedup_large_4t / 4.0;
  
  PCL_ALWAYS ("[Box, Minimum_Negative] 10K Samples, Res: 50, 4 Threads: [Runtime: %.3f, Speedup: %.3f, Overhead: %.3f, Efficiency: %.3f]\n", time_large_4t, speedup_large_4t, overhead_large_4t, efficiency_large_4t);
}

#if 0
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_BOX);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  std::vector<double> results;
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  results.push_back (cloud_out[2].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.5f, results[0]);
  EXPECT_EQ (1.0f, results[1]);
  EXPECT_EQ (2.0f, results[2]);
  EXPECT_EQ (3, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  results.clear ();
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  results.push_back (cloud_out[2].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.5f, results[0]);
  EXPECT_EQ (1.0f, results[1]);
  EXPECT_EQ (2.0f, results[2]);
  EXPECT_EQ (3, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box, Maximum_Negative)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MAX);
  lm.setNegative (true);
  lm.setLocalityType (lm.LT_BOX);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  EXPECT_EQ (1.0f, cloud_out[0].z);
  EXPECT_EQ (1, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (2.0f, cloud_out[0].z);
  EXPECT_EQ (1, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MAX);
  lm.setLocalityType (lm.LT_BOX);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  std::vector<double> results;
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  results.push_back (cloud_out[2].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.25f, results[0]);
  EXPECT_EQ (0.5f, results[1]);
  EXPECT_EQ (2.0f, results[2]);
  EXPECT_EQ (3, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  results.clear ();
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  results.push_back (cloud_out[2].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.25f, results[0]);
  EXPECT_EQ (0.5f, results[1]);
  EXPECT_EQ (1.0f, results[2]);
  EXPECT_EQ (3, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius, Minimum_Negative)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MIN);
  lm.setNegative (true);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (1, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  std::vector<double> results;
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  results.push_back (cloud_out[2].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.5f, results[0]);
  EXPECT_EQ (1.0f, results[1]);
  EXPECT_EQ (2.0f, results[2]);
  EXPECT_EQ (3, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius, Maximum_Negative)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MAX);
  lm.setNegative (true);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  EXPECT_EQ (1.0f, cloud_out[0].z);
  EXPECT_EQ (1, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MAX);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  std::vector<double> results;
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  results.push_back (cloud_out[2].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.25f, results[0]);
  EXPECT_EQ (0.50f, results[1]);
  EXPECT_EQ (2.0f, results[2]);
  EXPECT_EQ (3, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN, Minimum_Negative)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MIN);
  lm.setNegative (true);
  lm.setLocalityType (lm.LT_KNN);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (1, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (1, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_KNN);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  std::vector<double> results;
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  results.push_back (cloud_out[2].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.5f, results[0]);
  EXPECT_EQ (1.0f, results[1]);
  EXPECT_EQ (2.0f, results[2]);
  EXPECT_EQ (3, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  results.clear ();
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  results.push_back (cloud_out[2].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.5f, results[0]);
  EXPECT_EQ (1.0f, results[1]);
  EXPECT_EQ (2.0f, results[2]);
  EXPECT_EQ (3, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN, Maximum_Negative)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MAX);
  lm.setNegative (true);
  lm.setLocalityType (lm.LT_KNN);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  std::vector<double> results;
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (1.0f, results[0]);
  EXPECT_EQ (2.0f, results[1]);
  EXPECT_EQ (2, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  results.clear ();
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  results.push_back (cloud_out[2].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.5f, results[0]);
  EXPECT_EQ (1.0f, results[1]);
  EXPECT_EQ (2.0f, results[2]);
  EXPECT_EQ (3, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MAX);
  lm.setLocalityType (lm.LT_KNN);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  std::vector<double> results;
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (0.25f, results[0]);
  EXPECT_EQ (0.5f, results[1]);
  EXPECT_EQ (2, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (1, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Minimum_Negative)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (3);

  cloud_in[0].x = 0;   cloud_in[0].y = 0;   cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.5; cloud_in[1].y = 0.5; cloud_in[1].z = 1;
  cloud_in[2].x = 1.5; cloud_in[2].y = 1.5; cloud_in[2].z = 0.0;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MIN);
  lm.setNegative (true);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.25f);
  EXPECT_EQ (cloud_out.size (), 1);
  EXPECT_EQ (lm.getResolution (), 1.0f);

  lm.setResolution (2.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.0f);
  EXPECT_EQ (cloud_out.size (), 1);
  EXPECT_EQ (lm.getResolution (), 2.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (3);

  cloud_in[0].x = 0;   cloud_in[0].y = 0;   cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.5; cloud_in[1].y = 0.5; cloud_in[1].z = 1;
  cloud_in[2].x = 1.5; cloud_in[2].y = 1.5; cloud_in[2].z = 0.0;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MIN);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 1.0f);
  EXPECT_EQ (cloud_out.size (), 1);
  EXPECT_EQ (lm.getResolution (), 1.0f);

  lm.setResolution (2.0f);
  lm.filter (cloud_out);

  std::vector<double> results;
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (results[0], 0.25f);
  EXPECT_EQ (results[1], 1.0f);
  EXPECT_EQ (cloud_out.size (), 2);
  EXPECT_EQ (lm.getResolution (), 2.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Maximum_Negative)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (3);

  cloud_in[0].x = 0;   cloud_in[0].y = 0;   cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.5; cloud_in[1].y = 0.5; cloud_in[1].z = 1;
  cloud_in[2].x = 1.5; cloud_in[2].y = 1.5; cloud_in[2].z = 0.0;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MAX);
  lm.setNegative (true);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 1.0f);
  EXPECT_EQ (cloud_out.size (), 1);
  EXPECT_EQ (lm.getResolution (), 1.0f);

  lm.setResolution (2.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 1.0f);
  EXPECT_EQ (cloud_out.size (), 1);
  EXPECT_EQ (lm.getResolution (), 2.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (3);

  cloud_in[0].x = 0;   cloud_in[0].y = 0;   cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.5; cloud_in[1].y = 0.5; cloud_in[1].z = 1;
  cloud_in[2].x = 1.5; cloud_in[2].y = 1.5; cloud_in[2].z = 0.0;

  LocalMinMax<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MAX);
  lm.setNumberOfThreads (1);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.25f);
  EXPECT_EQ (cloud_out.size (), 1);
  EXPECT_EQ (lm.getResolution (), 1.0f);

  lm.setResolution (2.0f);
  lm.filter (cloud_out);

  std::vector<double> results;
  results.push_back (cloud_out[0].z);
  results.push_back (cloud_out[1].z);
  std::sort (results.begin (), results.end ());

  EXPECT_EQ (results[0], 0.0f);
  EXPECT_EQ (results[1], 0.25f);
  EXPECT_EQ (cloud_out.size (), 2);
  EXPECT_EQ (lm.getResolution (), 2.0f);
}
#endif
/* ---[ */
int
main (int argc, char** argv)
{
  int low = -100;
  int high = 100;

  cloud_in_10K.height = 1;
  cloud_in_10K.width = 10000;
  cloud_in_10K.is_dense = true;
  cloud_in_10K.resize (10000);

  for (int i = 0; i < cloud_in_10K.size (); i++)
  {
    cloud_in_10K[i].x = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
    cloud_in_10K[i].y = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
    cloud_in_10K[i].z = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
  }

  testing::InitGoogleTest (&argc, argv);
  int tmp = RUN_ALL_TESTS ();
  getchar ();
  return (tmp);
}
/* ]--- */

