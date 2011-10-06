#include <ros/ros.h>
#include <simon_says/Prob.h>
#include <opencv/cv.h>

cv::Mat sigma_, siginv_, mean_;


	/**
	 * C++ version 0.4 std::string style "itoa":
	 * Contributions from Stuart Lowe, Ray-Yuan Sheu,
	 * Rodrigo de Salvo Braz, Luc Gallant, John Maloney
	 * and Brian Hunt
	 */
	std::string itoa(int value, int base = 10) {
	
		std::string buf;
	
		// check that the base if valid
		if (base < 2 || base > 16) return buf;

		enum { kMaxDigits = 35 };
		buf.reserve( kMaxDigits ); // Pre-allocate enough space.
	
		int quotient = value;
	
		// Translating number to string with base:
		do {
			buf += "0123456789abcdef"[ std::abs( quotient % base ) ];
			quotient /= base;
		} while ( quotient );
	
		// Append the negative sign
		if ( value < 0) buf += '-';
	
		std::reverse( buf.begin(), buf.end() );
		return buf;
	}


bool prob_cb( simon_says::Prob::Request  &req,
							simon_says::Prob::Response &res )
{

	cv::Mat u(1,8,CV_64F);
	cv::Mat w(1,8,CV_64F);

	for( int i = 0; i < 8; i++ )
	{
		u.at<double>(0,i) = req.goal_state.position[i];
		w.at<double>(0,i) = req.current_state.position[i];
	}	
	

	cv::Mat wu = w-u-mean_;
	cv::Mat wut = wu.t();

	cv::Mat iterm = wu * siginv_ * wut;
	double mahal = sqrt(iterm.at<double>(0,0));
	double power = pow(2*M_PI,8/2.) * sqrt(fabs(cv::determinant(sigma_)));
	double leading_term = 1. / power;
	double prob = /*leading_term * */ exp( -0.5 * mahal );

	res.prob = prob;
	return true;
}


int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "prob_srv" );
  ros::NodeHandle nh;
	std::string prefix;
  ros::NodeHandle nh_priv("~");
	nh_priv.param( "prefix", prefix, std::string("") );
	ros::ServiceServer service = nh.advertiseService( "pose_prob", prob_cb );

	int num_lines = 0;
	std::vector<float> values;
	for( int i = 0; i < 6; i++ )
	{
		std::string filename = prefix + "/" + itoa(i) + std::string(".pose");
    ROS_INFO( "opening file: [%s]", filename.c_str() );
		FILE* infile = fopen(filename.c_str(),"r");
		
		std::vector<float> pose;
		for( int i = 0; i < 8; i++ )
		{
			float x = 0;
			fscanf( infile, "%f ", &x );
			//printf( "%0.2f ", x );
			pose.push_back(x);
		}
		//printf( "\n" );
		fscanf(infile,"\n");

		while( !feof( infile ) )
		{
			num_lines++;
			for( int i = 0; i < 8; i++ )
			{
				float x;
				fscanf( infile, "%f ", &x );
				//printf( "%0.2f ", x - pose[i] );
				values.push_back(x - pose[i]);
			}
			fscanf(infile,"\n");
			//printf ("\n" );
		}

		fclose(infile);
	}

	//printf ("\n\n\n\n======\n\n\n\n" );

	// calc covar matrix
	cv::Mat val_mat (num_lines,8,CV_64F);
	for( int i = 0; i < num_lines; i++ )
	{
		for( int j = 0; j < 8; j++ )
		{
			val_mat.at<double>(i,j) = values[i*8+j];
			//printf( "%0.2f ", val_mat.at<double>(i,j) );
		}
		//printf( "\n" );
	}

	cv::Mat covar(8,8,CV_64F), mean(8,1,CV_64F);
	cv::calcCovarMatrix(val_mat, covar, mean, CV_COVAR_NORMAL | CV_COVAR_ROWS | CV_COVAR_SCALE);
	//printf ("\n\n\n\n======\n\n\n\n" );

	for( int i = 0; i < 8; i++ )
	{
		for( int j = 0; j < 8; j++ )
		{
			printf( "%0.4f ", covar.at<double>(i,j));
		}
		printf( "\n" );
	}

	// we'll only use inverse of covariance matrix
	siginv_ = covar.inv(cv::DECOMP_SVD);
	mean_ = mean;
	sigma_ = covar;

  ros::spin();
  return 0; 
}

