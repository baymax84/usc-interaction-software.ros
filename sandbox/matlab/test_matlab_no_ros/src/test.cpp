/***************************************************************************
 *  src/test.cpp
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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
 *  * Neither the name of usc-ros-pkg nor the names of its
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

#include "mex.h"
#include <matlab/mat.h>
#include <iostream>

void printUsage()
{
	std::cout << "This function simply scales the given mat: test( scale, mat )\nmat must have non-zero dimensions and both arguments must be passed" << std::endl;
}

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
	// make sure we are reading exactly two arguments and returning no more than one value
	if( nrhs != 2 || nlhs > 1 ) return printUsage();

	// construct a matlab::Mat from the first argument; this stores an mxArray * and does not do any allocation
	// here, the resuling mat is 1x1
	// we then use the implicit cast operator to read a scalar ( identical to calling matlab::Mat::first() )
	const double scale = matlab::Mat<double>( prhs[0] );

	// construct a matlab::Mat from the second argument; this stores an mxArray * and does not do any allocation
	const matlab::Mat<double> input_mat( prhs[1] );

	// make sure the dimensions of the mat are at least 1x1
	if( input_mat.rows_ * input_mat.cols_ == 0 ) return printUsage();

//	std::cout << input_mat << std::endl;

	// construct a new matlab::Mat with the same dimensions as input_mat; this allocates a new mxArray
	// specifically, since we want to use doubles, it calls: mxCreateNumericArray( rows_, cols_, mxDOUBLE_CLASS, mxREAL )
	matlab::Mat<double> output_mat( input_mat.getDim() );

//	example of resizing the mat:
//	std::cout << output_mat << std::endl;
//	output_mat.resize( input_mat.rows_ + 1, input_mat.cols_ + 1 );
//	std::cout << output_mat << std::endl;

	// simple operation to show that everything is working; set every output value to be the corresponding input value scaled by the given scale
	for( unsigned int row = 0; row < input_mat.rows_; ++row )
	{
		for( unsigned int col = 0; col < input_mat.cols_; ++col )
		{
			// an operator[] is also available but the indexing is less intuitive for >1d matrices
			// it is useful for 1xN or Nx1 matrices, however
			output_mat.at( row, col ) = input_mat.at( row, col ) * scale;
		}
	}

	// uses the implicit cast operator for mxArray* to store output_mat value in the list of output values
	// this is identical to: plhs[0] = output_mat.getMat();
	plhs[0] = output_mat;
}
