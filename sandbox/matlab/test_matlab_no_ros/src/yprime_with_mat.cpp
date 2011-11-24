/*=================================================================
 *
 * YPRIME.C	Sample .MEX file corresponding to YPRIME.M
 *	        Solves simple 3 body orbit problem
 *
 * The calling syntax is:
 *
 *		[yp] = yprime(t, y)
 *
 *  You may also want to look at the corresponding M-code, yprime.m.
 *
 * This is a MEX-file for MATLAB.
 * Copyright 1984-2006 The MathWorks, Inc.
 *
 *=================================================================*/
/* $Revision: 1.10.6.4 $ */

/***************************************************************************
 *  src/yprime_with_mat.cpp
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

#include <math.h>
#include "mex.h"
#include <matlab/mat.h>

/* Input Arguments */

#define	T_IN	prhs[0]
#define	Y_IN	prhs[1]


/* Output Arguments */

#define	YP_OUT	plhs[0]

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

static	double	mu = 1/82.45;
static	double	mus = 1 - 1/82.45;

using namespace matlab;

static void yprime(
		   Mat<double> & yp,
		   const double & t,
 		   const Mat<double> & y
		   )
{
    double	r1,r2;

    //(void) t;     /* unused parameter */

    // Notice that we can index yp and y as if it was a double[]
    // Furthermore, to get proper row/col access, we can do:
    // yp.at( row, col )
    // to get the 4th row and the 1st column ( equivalent to yp[3] since yp must be a 4x1 or 1x4 ):
    // yp.at( 3, 0 ) or yp.at( 0, 3 )
    // the Mat::at() function can be used to read or assign values:
    // yp.at( 3, 0 ) = 5;

    r1 = sqrt((y[0]+mu)*(y[0]+mu) + y[2]*y[2]);
    r2 = sqrt((y[0]-mus)*(y[0]-mus) + y[2]*y[2]);

    /* Print warning if dividing by zero. */
    if (r1 == 0.0 || r2 == 0.0 ){
	mexWarnMsgTxt("Division by zero!\n");
    }

    yp[0] = y[1];
    yp[1] = 2*y[3]+y[0]-mus*(y[0]+mu)/(r1*r1*r1)-mu*(y[0]-mus)/(r2*r2*r2);
    yp[2] = y[3];
    yp[3] = -2*y[1] + y[2] - mus*y[2]/(r1*r1*r1) - mu*y[2]/(r2*r2*r2);
    return;
}

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[] )
{

    /* Check for proper number of arguments */

    if (nrhs != 2) {
	mexErrMsgTxt("Two input arguments required.");
    } else if (nlhs > 1) {
	mexErrMsgTxt("Too many output arguments.");
    }

    /* Check the dimensions of Y.  Y can be 4 X 1 or 1 X 4. */

	// wrap a matlab::Mat around Y_IN
	const Mat<double> y( Y_IN );

	// we know that T_IN is a 1x1 mat (ie just a double)
	// to get this double, we can do a few equivalent things
	// double t = Mat<double>( T_IN )[0]; // get item at 0th index
	// double t = Mat<double>( T_IN ).first(); // get the first item
	// double t = Mat<double>( T_IN ).at( 0, 0 ); // get the item at the 0th row and 0th column
	// or, we can use the implicit cast operator which will call matlab::Mat::first() for us:
	const double t = Mat<double>( T_IN );

    const int & m = y.rows_;
    const int & n = y.cols_;

    if( ( MAX( m, n ) != 4 ) || ( MIN( m, n ) != 1 ) )
    {
		mexErrMsgTxt( "YPRIME requires that Y be a 4 x 1 vector." );
    }

    /* Create a matrix for the return argument */
    // equivalent to:
    // Mat<double> yp( m, n );
    // Mat<double> yp( y.rows_, y.cols_ );
    Mat<double> yp( y.getDim() );

    /* Do the actual computations in a subroutine */
    yprime( yp, t, y );

	/* pass yp back to MATLAB */
	// equivalent to:
	// YP_OUT = yp.getMat();
	YP_OUT = yp;

    return;
}


