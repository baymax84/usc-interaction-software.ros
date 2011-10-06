#include <ros/ros.h>
#include <simon_says/ProbLR.h>
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


bool prob_cb( simon_says::ProbLR::Request  &req,
							simon_says::ProbLR::Response &res )
{

	cv::Mat u_l(1,4,CV_64F);
	cv::Mat w_l(1,4,CV_64F);
	cv::Mat u_r(1,4,CV_64F);
	cv::Mat w_r(1,4,CV_64F);

	for( int i = 0; i < 4; i++ )
	{
		u_l.at<double>(0,i) = req.goal_state.position[i];
		w_l.at<double>(0,i) = req.current_state.position[i];
		u_r.at<double>(0,i) = req.goal_state.position[i+4];
		w_r.at<double>(0,i) = req.current_state.position[i+4];
	}	
	

	cv::Mat wu_l = w_l-u_r-mean_;
	cv::Mat wut_l = wu_l.t();
	cv::Mat wu_r = w_r-u_r-mean_;
	cv::Mat wut_r = wu_r.t();

	cv::Mat iterm_l = wu_l * siginv_ * wut_l;
	cv::Mat iterm_r = wu_r * siginv_ * wut_r;

	double mahal = sqrt(iterm_l.at<double>(0,0));
	double power = pow(2*M_PI,4/2.) * sqrt(fabs(cv::determinant(sigma_)));
	double leading_term = 1. / power;
	double prob = /*leading_term * */ exp( -0.5 * mahal );

	res.left_prob = prob;

  // repeat for right
	mahal = sqrt(iterm_r.at<double>(0,0));
	prob = /*leading_term * */ exp( -0.5 * mahal );

  res.right_prob = prob;

	return true;
}


int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "prob_srv_lr" );
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
	cv::Mat val_mat (num_lines*2,4,CV_64F);
	for( int i = 0; i < num_lines*2; i++ )
	{
		for( int j = 0; j < 4; j++ )
		{
			val_mat.at<double>(i,j) = values[i*4+j];
			//printf( "%0.2f ", val_mat.at<double>(i,j) );
		}
		//printf( "\n" );
	}

	cv::Mat covar(4,4,CV_64F), mean(4,1,CV_64F);
	cv::calcCovarMatrix(val_mat, covar, mean, CV_COVAR_NORMAL | CV_COVAR_ROWS | CV_COVAR_SCALE);
	//printf ("\n\n\n\n======\n\n\n\n" );

	for( int i = 0; i < 4; i++ )
	{
		for( int j = 0; j < 4; j++ )
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

