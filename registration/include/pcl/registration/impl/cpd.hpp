/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc
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

#ifndef PCL_REGISTRATION_IMPL_CPD_HPP_
#define PCL_REGISTRATION_IMPL_CPD_HPP_

#include <Eigen/Core>

#include <pcl/registration/boost.h>
#include <pcl/correspondence.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::CoherentPointDrift<PointSource, PointTarget, Scalar>::computeTransformation (
    PointCloudSource &output, const Matrix4 &guess)
{
  
  int dimensions = 3; // x, y, z
  int input_length = (int) indices_->size (); // M
  int target_length = (int) target_->points.size (); // N

  // convert input data to eigen data
  Eigen::MatrixXf input_matrix (dimensions, input_length);
  Eigen::MatrixXf input_centered (dimensions, input_length);
  Eigen::MatrixXf output_matrix (dimensions, input_length);
  Eigen::MatrixXf target_matrix (dimensions, input_length);

  // copy the input data to the intput and output matrices
  for (int i = 0; i < input_length; ++i)
  {
    input_matrix (0, i) = output_matrix (0, i) = input_->points[(*indices_)[i]].x;
    input_matrix (1, i) = output_matrix (1, i) = input_->points[(*indices_)[i]].y;
    input_matrix (2, i) = output_matrix (2, i) = input_->points[(*indices_)[i]].z;
  }

  // copy the target data
  for (int i = 0; i < target_length; ++i)
  {
    target_matrix (0, i) = target_->points[i].x;
    target_matrix (1, i) = target_->points[i].y;
    target_matrix (2, i) = target_->points[i].z;
  }

  Eigen::MatrixXf input_mean (dimensions, 1);
  Eigen::MatrixXf target_mean (dimensions, 1);
  input_mean.setZero ();
  target_mean.setZero ();

  float input_scale = 1.0f;
  float target_scale = 1.0f;

  // normalize to zero mean and unit variance
  if (normalize_)
  {
    // we lose some precision here due to the use of float over double
    input_mean = input_matrix.rowwise ().mean ();
    target_mean = target_matrix.rowwise ().mean ();

    input_matrix = input_matrix - input_mean.replicate (1, input_length);
    target_matrix = target_matrix - target_mean.replicate (1, target_length);

    input_scale = std::sqrt (input_matrix.array ().square ().matrix ().sum () / input_length);
    target_scale = std::sqrt (target_matrix.array ().square ().matrix ().sum () / target_length);

    input_matrix = input_matrix / input_scale;
    target_matrix = target_matrix / target_scale;
  }

  // initialization stuff
  if (sigma_squared_ <= 0)
  {
    float input_trace = (input_matrix * input_matrix.transpose ()).trace ();
    float target_trace = (target_matrix * target_matrix.transpose ()).trace ();

    sigma_squared_ = (input_length * target_trace + target_length * input_trace - (2.0f * target_matrix.colwise ().sum () * input_matrix.colwise ().sum ().transpose ())) / (input_length * target_length * dimensions);
  }

  float sigma_squared_init = sigma_squared_;

  // initialize the rotation matrix and scaling coefficient
  Eigen::MatrixXf rot_matrix = Eigen::MatrixXf::Identity(dimensions, dimensions);
  Eigen::MatrixXf B;
  int iteration = 0;
  float new_tolerance = tolerance_ + 10.0f;
  float new_likelihood;
  float old_likelihood;
  
  if (registration_mode_ == RM_RIGID)
    new_likelihood = 0.0f;
  else
    new_likelihood = 1.0f;

  Eigen::VectorXf target_prob_sum (target_length); // Pt1
  Eigen::VectorXf input_prob_sum (input_length); // P1
  Eigen::VectorXf input_prob (input_length);
  std::vector <float> cur_point (dimensions); // temp_x

  float last_sigma_squared = sigma_squared_;
  float scale = 1;
  Eigen::VectorXf trans;

  // loop until convergence
  while ((iteration++ < max_iterations_) && (new_tolerance > tolerance_) && (sigma_squared_ > 10*std::numeric_limits<float>::epsilon ()))
  {

    // clear our probability vectors
    target_prob_sum.setZero ();
    input_prob_sum.setZero ();
    input_prob.setZero ();
    input_centered.setZero ();

    // save the previous likelihood estimate
    old_likelihood = new_likelihood;

    // Expectation Step
    if (use_fgt_)
    {
      // TODO: Implement FGT
      return;
    }
    else
    {

      float ksig = -2.0 * sigma_squared_;
      float outlier = (outlier_weight_ * input_length * std::pow (-ksig * M_PI, 0.5 * dimensions)) / ((1.0 - outlier_weight_) * target_length);
        
      // reset our likelihood estimate
      new_likelihood = 0.0f;

      // for each point in the target set
      for (int n = 0; n < target_length; ++n) 
      {
      
        float prob_sum = 0.0;

        // for each point in the input set
        for (int m = 0; m < input_length; ++m) 
        {
          float dist_squared = 0.0;
          
          // for each dimension (x, y, z)
          for (int d = 0; d < dimensions; ++d) 
          {
            float diff = target_matrix (d, n) - output_matrix (d, m);  
            diff = diff * diff;
            dist_squared += diff;
          }
          
          input_prob (m) = std::exp (dist_squared / ksig);
          prob_sum += input_prob (m);
        }
      
        prob_sum += outlier;
      
        target_prob_sum (n) = 1 - (outlier / prob_sum);
      
        // for each dimension
        for (int d = 0; d < dimensions; ++d) 
        {
          cur_point[d] = target_matrix (d, n) / prob_sum;
        }
      
        // for each point in the input set
        for (int m = 0; m < input_length; ++m) 
        {
         
          input_prob_sum (m) += input_prob (m) / prob_sum;
          
          // for each dimension
          for (int d = 0; d < dimensions; ++d) 
          {
            input_centered (d, m) += cur_point[d] * input_prob[m];
          }
        }
      
        new_likelihood += -std::log (prob_sum);     
      }

      new_likelihood += (float)dimensions * (float)target_length * std::log (sigma_squared_) / 2.0f;
    }

    // keep track of the % change in likelihood
    new_tolerance = std::abs ((new_likelihood - old_likelihood) / new_likelihood);

    // precompute
    float target_prob_total = target_prob_sum.sum ();
    Eigen::VectorXf mu_x = target_matrix * target_prob_sum / target_prob_total;
    Eigen::VectorXf mu_y = input_matrix * input_prob_sum / target_prob_total;

    // solve for rotation, scaling, translation and sigma_squared
    Eigen::MatrixXf transform_matrix = input_centered * input_matrix.transpose () - target_prob_total * (mu_x * mu_y.transpose ());

    if (registration_mode_ == RM_RIGID)
    {

      Eigen::JacobiSVD <Eigen::MatrixXf> svd (transform_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
      Eigen::MatrixXf c_matrix = Eigen::MatrixXf::Identity(dimensions, dimensions);

      if (use_strict_rotation_)
        c_matrix (dimensions-1, dimensions-1) = (svd.matrixU () * svd.matrixV ().transpose ()).determinant ();

      rot_matrix = svd.matrixU () * c_matrix * svd.matrixV ().transpose ();

      last_sigma_squared = sigma_squared_;

      if (estimate_scaling_)
      {
        float t1 = (svd.singularValues ().matrix ().transpose () * c_matrix).sum ();
        float t2 = ((input_matrix.array ().square ().matrix () * input_prob_sum.replicate (dimensions, 1)).colwise ().sum ()).sum();
        float t3 = target_prob_total * (float)(mu_y.transpose () * mu_y);
        scale = t1 / (t2 - t3);

        t1 = ((target_matrix.array ().square ().matrix () * target_prob_sum.replicate (dimensions, 1)).colwise ().sum ()).sum();
        t2 = target_prob_total * (float)(mu_x.transpose () * mu_x);
        t3 = scale * (svd.singularValues ().matrix ().transpose () * c_matrix).sum ();
        sigma_squared_ = std::abs(t1 - t2 - t3) / (target_prob_total * dimensions);
      }
      else
      {
        float t1 = ((target_matrix.array ().square ().matrix () * target_prob_sum.replicate (dimensions, 1)).colwise ().sum ()).sum();
        float t2 = target_prob_total * (float)(mu_x.transpose () * mu_x);
        float t3 = ((input_matrix.array ().square ().matrix () * input_prob_sum.replicate (dimensions, 1)).colwise ().sum ()).sum();
        float t4 = target_prob_total * (float)(mu_y.transpose () * mu_y);
        float t5 = 2 * (svd.singularValues ().matrix ().transpose () * c_matrix).sum ();
        sigma_squared_ = std::abs((t1 - t2 + t3 - t4 - t5) / (target_prob_total * dimensions));
      }

      trans = mu_x - scale * rot_matrix * mu_y;

      // update the GMM centroids
      output_matrix = scale * rot_matrix * input_matrix + trans.replicate (1, input_length);
    }
    else if (registration_mode_ == RM_AFFINE)
    {
      //B2=(Y.*repmat(P1,1,D))'*Y-Np*(mu_y*mu_y');
      Eigen::MatrixXf t1 = (input_matrix.transpose ().cwiseProduct (input_prob_sum.replicate (1, dimensions))).transpose () * input_matrix.transpose ();
      Eigen::MatrixXf t2 = target_prob_total * (mu_y * mu_y.transpose ());
      Eigen::MatrixXf B2 = t1 - t2;
      
      //B=B1/B2; % B= B1 * inv(B2);
      B = transform_matrix * B2.inverse ();

      last_sigma_squared = sigma_squared_;

      //sigma2=abs(sum(sum(X.^2.*repmat(Pt1,1,D)))- Np*(mu_x'*mu_x) -trace(B1*B'))/(Np*D); 
      output_matrix = scale * rot_matrix * input_matrix + trans.replicate (1, input_length);
      float f1 = ((target_matrix.array ().square ().matrix () * target_prob_sum.replicate (dimensions, 1)).colwise ().sum ()).sum();
      float f2 = target_prob_total * (float)(mu_x.transpose () * mu_x);
      float f3 = (transform_matrix.transpose () * B).trace ();
      sigma_squared_ = std::abs(f1 - f2 - f3) / (target_prob_total * dimensions);

      trans = mu_x - B * mu_y;

      // update the GMM centroids
      output_matrix = input_matrix * B.transpose () + trans.replicate (1, input_length);
    }
  }

  // denormalize
  if (normalize_)
  {
    scale = scale * target_scale / input_scale;
    trans = target_scale * trans + target_mean - scale * (rot_matrix * input_mean);
    Eigen::MatrixXf tmp = target_mean.replicate (1, input_length);
    output_matrix = output_matrix * target_scale + target_mean.replicate (1, input_length);

    if (registration_mode_ == RM_AFFINE)
    {
      rot_matrix = scale * B;
      scale = 1.0f;
    }
  }

  // copy output matrix to output point cloud
  for (int i = 0; i < input_length; ++i)
  {
    output.points[i].x = output_matrix (0, i);
    output.points[i].y = output_matrix (1, i);
    output.points[i].z = output_matrix (2, i);
  }
}

#endif /* PCL_REGISTRATION_IMPL_CPD_HPP_ */
