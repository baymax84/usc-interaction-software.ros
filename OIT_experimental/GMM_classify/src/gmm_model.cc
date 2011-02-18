#include <ros/ros.h>
#include <feature_extractor/CPRWFeatureVector.h>
#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>


class DaveEM : public CvEM {
  public:
  double get_likelihood() { return log_likelihood; }

};

int main( int argc, char** argv )
{
	ros::init(argc,argv,"gmm_model");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

  // open file
  FILE* model_file;

	// get model name from args
	std::string model_filename;
	nh_priv.param( "model_filename", model_filename, std::string("train.out"));

	model_file = fopen( model_filename.c_str(), "r" );

	int d = 0;
	int nsamples = 0;
	fscanf( model_file, "%d %d\n", &nsamples, &d );

	CvMat* samples = cvCreateMat( nsamples, d, CV_32FC1 );
	CvMat* labels = cvCreateMat( nsamples, 1, CV_32SC1 );

	for( int i = 0; i < nsamples; i++ )
	{
		for( int j = 0; j < d; j++ )
		{
			float x = 0.0;
			fscanf( model_file, "%f ", &x );
			((float*)(samples->data.ptr+samples->step*i))[j] = x;
			//printf( "%4.6f ", x );
		}
		fscanf( model_file, "\n" );
		//printf( "\n" );
	}

	// zscore(samples)
	cv::Scalar* mean = new cv::Scalar[d];
	cv::Scalar* std = new cv::Scalar[d];
	
	cv::Mat C(samples);

	for( int i = 0; i < d; i++ )
	{
		cv::meanStdDev( C.col(i), mean[i], std[i] );
		//printf( "%d: %lf %lf\n", i, mean[i][0], std[i][0] );
		cv::Mat tmpcol(nsamples, 1, CV_32FC1);
		tmpcol = C.col(i);
		tmpcol = tmpcol - mean[i];
		tmpcol = tmpcol * (1./std[i][0]);
		//cv::Scalar m, s;
		//cv::meanStdDev( C.col(i), m, s );
		//printf( "%d: %lf %lf\n", i, m[0], s[0] );
	}

	CvMat trainData = C;

  CvEMParams params;
  // initialize model's parameters
  params.covs      = NULL;
  params.means     = NULL;
  params.weights   = NULL;
  params.probs     = NULL;
  params.cov_mat_type       = CvEM::COV_MAT_GENERIC;
  params.start_step         = CvEM::START_AUTO_STEP;
  params.term_crit.max_iter = 100;
  params.term_crit.epsilon  = 0.01;
  params.term_crit.type     = CV_TERMCRIT_EPS;//CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;
	// load groups from file

	CvMat* truths = cvCreateMat( nsamples, 1, CV_32SC1 );
	for( int i = 0; i < nsamples; i++ )
	{
		int x = 0;
		fscanf( model_file, "%d\n", &x );
		truths->data.i[i] = x;
		//CV_MAT_ELEM( *truths, int, i, 0 ) = x;
	}

	// read in test Data as well
	int ntest = 0;
	fscanf( model_file, "%d\n", &ntest );
	CvMat* test_samples = cvCreateMat( ntest, d, CV_32FC1 );
	CvMat* test_labels = cvCreateMat( ntest, 1, CV_32SC1 );
	CvMat* test_truths = cvCreateMat( ntest, 1, CV_32SC1 );

	for( int i = 0; i < ntest; i++ )
	{
		for( int j = 0; j < d; j++ )
		{
			float x = 0.0;
			fscanf( model_file, "%f ", &x );
			test_samples->data.fl[i*d+j] = x;
		}
		float x = 0;
		fscanf( model_file, "%f \n", &x );
		test_truths->data.i[i] = (int) round(x);
		//printf( "truth: %d\n", (int) round(x) );
	}

	// Standardize test_samples to the normalization of the training Data

	cv::Mat tmp(test_samples);
	for( int i = 0; i < d; i++ )
	{
		cv::Mat tmpcol(ntest, 1, CV_32FC1);
		tmpcol = tmp.col(i);
		tmpcol = tmpcol - mean[i];
		tmpcol = tmpcol * (1./std[i][0]);
	}
	
	CvMat testData = tmp;

	double vals[200];
	double smallval = DBL_MAX;
	int idx = 40;
	//int idx = 34;
/*
	for( int i = 2; i < 100; i++ ) 
	{
	  DaveEM em_model;
		params.nclusters = i;
		em_model.train( &trainData, 0, params, labels );
		vals[i] = em_model.get_likelihood();
		double add = (i-1)*log(nsamples)/2.0 + ((d*d+3*d)/4.0)*i*log(nsamples);
		printf( "%4d: %8.2lf %8.4lf %8.4lf\n", i, vals[i], add, -vals[i] + add );
		vals[i] = -vals[i]+add;
		if( vals[i] < smallval ) 
		{
			smallval = vals[i];
			idx = i;
		}
	}


	printf( "smallest is: %d with: %lf\n", idx, smallval );
*/

	DaveEM em_model;
	params.nclusters = idx;
	em_model.train(&trainData,0,params,labels);

	
	// gmm model is trained, but classifier needs to be trained now


	// generate pC and pO
	double pC[100];
	double* pO = new double[params.nclusters];

	int num_groups = 0;
	for( int i = 0; i < 100; i++ ) pC[i] = 0;
	for( int i = 0; i < truths->rows; i++ )
	{
		int x = truths->data.i[i];
		pC[x-1]+= 1;
		if( x > num_groups ) num_groups = x;
	}
	printf( "pC: \n" );
	for( int i = 0; i < num_groups; i++ )
	{
		pC[i] /= truths->rows;
		//printf( "%d: %0.12f\n", i, pC[i]*100. );
	}

	printf( "labels: %d clusters: %d\n", labels->rows, params.nclusters );

	for( int i = 0; i < params.nclusters; i++ ) pO[i] = 0;
	for( int i = 0; i < labels->rows; i++ )
	{
		int x = labels->data.i[i];
		pO[x] += 1;	
	}
	printf( "pO: \n" );
	for( int i = 0; i < params.nclusters; i++ )
	{
		pO[i] /= labels->rows;
		//printf( "%d: %0.12f\n", i, pO[i]*100. );
	}

	CvMat* pOC = cvCreateMat( params.nclusters, num_groups, CV_32FC1 );
	// pOC
	int *clus_dist = new int[params.nclusters];
	for( int i = 0; i < params.nclusters; i++ )
	{
		for( int j = 0; j < num_groups; j++ ) pOC->data.fl[i*num_groups+j]=0;
		clus_dist[i] = 0;
	}

	for( int j = 0; j < labels->rows; j++ )
	{
		int cluster = labels->data.i[j];
		int truth = truths->data.i[j]-1;
		pOC->data.fl[cluster*num_groups+truth] += 1.;
		clus_dist[cluster]++;
	}
	for( int i = 0; i < params.nclusters; i++ )
	{
		for( int j = 0; j < num_groups; j++ )
		{
			pOC->data.fl[i*num_groups+j] /= (double) (clus_dist[i]);
			//printf( "%4.2f ", pOC->data.fl[i*num_groups+j] * 100. );
		}
		//printf( "\n" );
	}

	// classifier is trained... start listening for data

	ros::Rate loop_rate(15);
	while( ros::ok() )
	{

	}

	// cluster test data using model (pO)
	CvMat* po = cvCreateMat(ntest,params.nclusters, CV_32FC1 );
	CvMat* pot = cvCreateMat(1,params.nclusters,CV_32FC1);
	cv::Mat testDataMat(&testData);

	int ntiles = ntest / 30;

	for( int i = 0; i < ntest; i++ )
	{
		CvMat testRow = testDataMat.row(i);
		em_model.predict( &testRow, pot );
		for( int j = 0; j < params.nclusters; j++ )
		{
			po->data.fl[i*params.nclusters+j] = pot->data.fl[j];
		}
	}

	
	CvMat* pCO = cvCreateMat( ntest, num_groups, CV_32FC1 );
	cvMatMul( po, pOC, pCO );
	cv::Mat pCOmat(pCO);

	int ** confusion = new int*[num_groups];
	for( int i = 0; i < num_groups; i++ ) 
	{
		confusion[i] = new int[num_groups];
		for( int j = 0; j < num_groups; j++ )
			confusion[i][j] = 0;
	}

	for( int i = 0; i < ntiles; i++ )
	{
		const int tile_min = i*30;
		const int tile_max = (i+1)*30;

		int* true_hist = new int[num_groups];
		double* vis_hist = new double[num_groups];
		for( int j = 0; j < num_groups; j++ ) 
		{
			true_hist[j] = 0;
			vis_hist[j] = 0;
		}
		// get tile's true class
		for( int j = tile_min; j < tile_max; j++ )
		{
			true_hist[test_truths->data.i[j]-1]++;
/*		
			cv::Mat matrow = pCOmat.row(j);
			CvMat row = matrow;
			
			CvMat obs_dist = row * 
*/
			for( int k = 0; k < num_groups; k++ )
			{
				vis_hist[k] += pCO->data.fl[j*num_groups+k];
			}
		}

		int max = 0;
		int true_value = -1;
		for( int j = 0; j < num_groups; j++ )
		{
			//printf( "%d ", true_hist[j] );
			if( true_hist[j] > max ) 
			{
				max = true_hist[j];
				true_value = j;
			}
		}
		//printf( "\n" );
		// get pCO for tile
		int obs_value = -1;
		double obs_max = -DBL_MAX;
		for( int j = 0; j < num_groups; j++ )
			if( vis_hist[j] > obs_max )
			{
				obs_max = vis_hist[j];
				obs_value = j;
			}

		if( true_value == 1 ) true_value = 2;
		if( obs_value == 1 ) obs_value = 2;
		//printf( "true_value: %d obs_value: %d\n", true_value, obs_value );
		confusion[true_value][obs_value] ++;
	}

/*
	for( int i = 0; i < num_groups; i++ )
	{
		for( int j = 0; j < num_groups; j++ )
		{
			//printf( "%6d", confusion[j][i] );
		}
		//printf( "\n" );
	}

	double all = 0.0;
	double correct = 0.0;

	for( int i = 0; i < num_groups; i+=2 )
	{
		// get total
		double total = 0;
		for( int j = 0; j < num_groups; j+=2 )
		{
			total += confusion[i][j];
 		}
		//printf( "%4.2f ", 100.0 * confusion[i][i] / total );
		correct += confusion[i][i];
		all += total;
	}
	//printf( "\n" );

	printf( "correct: %0.2f\n", 100.0*correct / all );
*/
	fclose(model_file);


	return 0;
}
