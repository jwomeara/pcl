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

#ifndef PCL_FILTERS_LOCAL_MODIFY_OMP_H_
#define PCL_FILTERS_LOCAL_MODIFY_OMP_H_

#include <pcl/filters/local_modify.h>

namespace pcl
{
  /** \brief LocalModify modifies the z dimension of each point to be equal to the local min/max/mean/median
    *
    * The LocalModify class analyzes each point and sets it's z dimension to
    * the value corresponding to the local min/max/mean/median with respect 
    * to their neighbors.  Box, radius, K-Nearest Neighbors and grid locality 
    * types are supported.
    *
    * \author James W. O'Meara
    * \ingroup filters
    */
  template <typename PointT>
  class LocalModifyOMP: public LocalModify<PointT>
  {

    public:
      /** \brief Empty constructor. */
      LocalModifyOMP (unsigned int threads = 0) :
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
      typedef typename LocalModify<PointT>::PointCloud PointCloud;

      using LocalModify<PointT>::input_;
      using LocalModify<PointT>::indices_;
      using LocalModify<PointT>::filter_name_;
      using LocalModify<PointT>::getClassName;
      using LocalModify<PointT>::searcher_;
      using LocalModify<PointT>::octree_;
      using LocalModify<PointT>::radius_;
      using LocalModify<PointT>::num_neighbors_;
      using LocalModify<PointT>::resolution_;
      using LocalModify<PointT>::inverse_resolution_;
      using LocalModify<PointT>::locality_type_;
      using LocalModify<PointT>::stat_type_;

      /** \brief The number of threads the scheduler should use. */
      unsigned int threads_;

      /** \brief Filtered results are indexed by an indices array using grid locality.
        * \param[out] indices The resultant indices.
        */
      void
      applyGridFilter (PointCloud &output);

      /** \brief Filtered results are indexed by an indices array using local locality.
        * \param[out] indices The resultant indices.
        */
      void
      applyLocalFilter (PointCloud &output);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/local_modify_omp.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_LOCAL_MODIFY_OMP_H_

