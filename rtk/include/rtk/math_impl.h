/***************************************************************************
 *  include/rtk/math_impl.h
 *  --------------------
 *
 *  Copyright (c) 2012, Dylan J. Foster
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of rtk nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef RTK_MATHIMPL_H_
#define RTK_MATHIMPL_H_

#include <math.h>

#include <vector>

#include <rtk/rtk.h>

namespace rtk
{
  namespace spatial
  {

    /** Get a vector representing the difference between two vectors 
     * 
     * @param vec1 vector from which return value originates 
     * @param vec2 vector to which return value points
     * 
     * @return Vector from vec1 to vec2 (vec2 - vec1)
     */
    _Vector getDistance(const _Vector & vec1, const _Vector & vec2);
    

    /** Get euclidian distance between two points in 3d space (represented as vectors)
     * 
     * @param vec1 First point
     * @param vec2 Second point
     * 
     * @return Euclidian distance between the two points
     */
    double getDistanceBetween(const _Vector & vec1, const _Vector & vec2);
    

    /**  Get euclidian distance squared between two points in 3d space (represented as vectors)
     * 
     * @param vec1 First point
     * @param vec2 Second point
     * 
     * @return Euclidian distance between the two points, squared
     */
    double getDistanceBetween2(const _Vector & vec1, const _Vector & vec2);
    
    
    /** Returns the length of a chain, calculated by adding together the lengths of the segments
     * 
     * @param chain chain for which the length will be calculated
     * 
     * @return returns the length of the chain in meters
     */
    double getLength(const _Chain & chain);

       
    /** Multiply the length of a chain by a scalar (segment-wise). 
     * 
     * @param lhs Chain to be scaled
     * @param rhs Amount by which to scale
     * 
     * @return Scaled chain
     */
    _Chain operator * (const _Chain& lhs, const double & rhs);

    
    /* TODO: Add a description or remove this function */
    _Chain scaleChain (const _Chain & chain, const double & scale);


    /** Normalize a chain to unit length
     * 
     * @param chain Chain to be normalized
     * 
     * @return Normalized chain
     */
    _Chain normalize(const _Chain& chain);
    
    
    /** Returns the length of a chain (represented as an array of frames), calculated by adding together the lengths of the segments
     * 
     * @param chain chain for which the length will be calculated
     * 
     * @return returns the length of the chain in meters
     */
    double getLength(const _FrameArray & chain);

    
    /** Multiply the length of a chain by a scalar (segment-wise). 
     * 
     * @param lhs Chain to be scaled (represented as an array of frames)
     * @param rhs Amount by which to scale
     * 
     * @return Scaled chain
     */
    _FrameArray operator * (const _FrameArray & lhs, const double & rhs);

      
    /** Normalize a chain to unit length
     * 
     * @param chain Chain to be normalized (represented as an array of Frames)
     * 
     * @return Normalized chain
     */
    _FrameArray normalize(const _FrameArray & chain);

    /// TODO: Write a full description or remove this function
    _FrameArray scaleChain(const _FrameArray & chain, const double & scale);
    
    /** 
     * Apply a transformation (Frame) to every frame in an array of frames
     * 
     * @param chain Chain (represented as an array of frames) to which transformation is applied
     * @param transform_frame Transform to apply
     */
    void transform(_FrameArray & chain, const _Frame & transform_frame);


    /** Translate a chain such that the most parent joint is located at the origin (0,0,0)
     * Note: This function assumes that the first joint in the array is the most parent joint
     * 
     * @param chain Chain (represented as an array of frames) to be translated
     */
    void translateToOrigin(_FrameArray & chain);

    
    /** Perform forward kinematics on a chain using a set of joint angles
     * 
     * @param chain Chain to be posed
     * @param angles Angles of the chain's joints in the new pose
     * @param solved_pose Array of frames where the solved pose will be stored. 
     *
     * @return Non-zero if error, zero otherwise.
     */
    int forwardKinematics(const _Chain & chain, const _JntArray & angles, _FrameArray & solved_pose);

    
    /** Calculated Sum of squared-distances between all of the frames in two arrays.
     * 
     * @param pose1 First array of frames to be compared.
     * @param pose2 Second array of frames to be compared.
     * 
     * @return Distance between the two arrays of frames in a squared-distance sense;
     */
    double totalSquaredDistance(const _FrameArray & pose1, const _FrameArray & pose2);


    /** Given an array of frames, compute the center-of-mass of their positions
     * All frames are given the same weight
     * 
     * @param frames The array of frames of which centroid will be computed
     * 
     * @return Frame to the centroid (unit rotation)
     */
    _Frame getCentroid(const _FrameArray & frames);

    /** Take a chain represented as an array of Frames and lie it straight   
     * along the x axis
     * 
     * @param frames The array of Frames being straightened
     * 
     * @return FrameArray The straightened chain
     */
    _FrameArray straightenFrameArray(const _FrameArray & frames);
    
    
  } // spatial
} // rtk

#endif // RTK_MATHIMPL_H_
