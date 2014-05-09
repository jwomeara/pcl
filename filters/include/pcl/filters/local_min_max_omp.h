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

#ifndef PCL_FILTERS_LOCAL_MIN_MAX_OMP_H_
#define PCL_FILTERS_LOCAL_MIN_MAX_OMP_H_

#include <pcl/filters/local_min_max.h>

namespace pcl
{
  /** \brief LocalMinMax downsamples the cloud, by eliminating points that are locally minimal/maximal.
    *
    * The LocalMinMax class analyzes each point and removes those that are
    * found to be locally minimal/maximal with respect to their neighbors.  
    * Box, radius, K-Nearest Neighbors and grid locality types are supported.
    * The comparison can be made in the x, y, or z dimensions.
    *
    * \author James W. O'Meara
    * \ingroup filters
    */
  template <typename PointT>
  class LocalMinMaxOMP: public LocalMinMax<PointT>
  {

    public:

      /** \brief Empty constructor. */
      LocalMinMaxOMP (bool extract_removed_indices = false,
                      unsigned int threads = 0) :
        LocalMinMax<PointT>::LocalMinMax (extract_removed_indices),
        threads_ (threads)
      {

        // number of threads defaults to the number of processors
        if (threads_ > 0)
          omp_set_num_threads (threads_);
        else
          omp_set_num_threads (omp_get_num_procs ());
      }

      /** \brief Set the number of threads to use.
        * \param threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      inline void 
      setNumberOfThreads (unsigned int threads = 0) 
      { 
        threads_ = threads; 
        if (threads_ > 0)
          omp_set_num_threads (threads_);
      }

    protected:
      typedef typename LocalMinMax<PointT>::PointCloud PointCloud;

      using LocalMinMax<PointT>::input_;
      using LocalMinMax<PointT>::indices_;
      using LocalMinMax<PointT>::filter_name_;
      using LocalMinMax<PointT>::getClassName;
      using LocalMinMax<PointT>::negative_;
      using LocalMinMax<PointT>::extract_removed_indices_;
      using LocalMinMax<PointT>::removed_indices_;
      using LocalMinMax<PointT>::searcher_;
      using LocalMinMax<PointT>::octree_;
      using LocalMinMax<PointT>::radius_;
      using LocalMinMax<PointT>::num_neighbors_;
      using LocalMinMax<PointT>::resolution_;
      using LocalMinMax<PointT>::inverse_resolution_;
      using LocalMinMax<PointT>::locality_type_;
      using LocalMinMax<PointT>::stat_type_;
      using LocalMinMax<PointT>::model_;

      /** \brief The number of threads the scheduler should use. */
      unsigned int threads_;

      /** \brief Filtered results are indexed by an indices array using grid locality.
        * \param[out] indices The resultant indices.
        */
      void
      applyGridFilter (std::vector<int> &indices);

      /** \brief Filtered results are indexed by an indices array using local locality.
        * \param[out] indices The resultant indices.
        */
      void
      applyLocalFilter (std::vector<int> &indices);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/local_min_max_omp.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_LOCAL_MIN_MAX_OMP_H_

