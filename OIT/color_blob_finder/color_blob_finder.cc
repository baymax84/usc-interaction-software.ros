#include "ros/ros.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "image_transport/image_transport.h"
#include "image_geometry/pinhole_camera_model.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include "sensor_msgs/fill_image.h"

//#include "contour_fcns.h"
#include "color_finder.h"

//#include "ray.h"

using namespace std;

class ImageSplitter
{
public:
  ros::NodeHandle n;
private:
  ros::Publisher hsv_pub;
  ros::Publisher foreground_pub;

  sensor_msgs::CvBridge img_bridge_;
  sensor_msgs::Image img_;
  IplImage *hsv_img_, *disp_;
  //IplImage *foreground_img_, *background_, *foreground_thresh_, *foreground_tmp_, *foreground_, *foreground_mask_;

	image_geometry::PinholeCameraModel pcam_;
	bool first;
  int display;

  IplConvKernel *kernel_;
  CvMemStorage* storage_;
  vector<ColorFinder> color_finders_;
  vector<CvScalar> colors_;

  public:

  void image_cb( const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg )
  {
		{
      sensor_msgs::CameraInfo cam2 = *infomsg;
      cam2.header.frame_id = "/ovh";
      pcam_.fromCameraInfo(cam2);
      first = false;
		}

    sensor_msgs::Image img_msg = *(msg.get());
    if( img_bridge_.fromImage( img_msg, "bgr8" ) )
    {
      ros::Time img_time = msg->header.stamp;
      IplImage* frame = img_bridge_.toIpl();

      if( frame->width != hsv_img_->width )
      {
        ROS_WARN( "resizing images to: [%d,%d]", cvGetSize(frame).width, cvGetSize(frame).height );
        //resize images
        cvReleaseImage( &hsv_img_ );
        cvReleaseImage( &disp_ );
        //cvReleaseImage( &foreground_img_ );

        hsv_img_ = cvCreateImage( cvGetSize(frame), 8, 3 );
        disp_ = cvCreateImage( cvGetSize(frame), 8, 3 );
        //foreground_img_ = cvCreateImage( cvGetSize(frame), 8, 1 );
      }
      cvCopy( frame, disp_ );
      cvCvtColor( frame, hsv_img_, CV_BGR2HSV );
      
			
			//cv::Mat matFrame(frame);
			//cv::Mat rectified;
			//pcam_.rectifyImage(matFrame, rectified );

			

      // color finders
      int c = 0;
      for( vector<ColorFinder>::iterator i = color_finders_.begin();
            i != color_finders_.end(); i++ )
      {
        // color probability images
        i->image_cb( hsv_img_ );
        // find contours
        i->find_blobs(img_msg.header.stamp);
        // publish blobs

        // render blobs
        vector<oit_msgs::Blob> blobs = i->get_blobs();
        for( unsigned int j = 0; j < blobs.size(); j++ )
        {
          oit_msgs::Blob b = blobs[j];
          cvRectangle( disp_, cvPoint( b.x, b.y ), 
                              cvPoint( b.x+b.width, b.y+b.height ),
                              colors_[c], 1 );         
        }
        c++;
      }

      // robot detector


			//cv::imshow("rectified", rectified);
      // foreground detector
     /* 
        // difference image
      cvAbsDiff( frame, background_, foreground_tmp_ );
      cvZero( foreground_thresh_ );
      cvCvtColor( foreground_tmp_, foreground_thresh_, CV_BGR2GRAY );
        // threshold
      cvThreshold(foreground_thresh_,foreground_thresh_,15,1,CV_THRESH_BINARY);

        // contours

      cvErode( foreground_thresh_, foreground_thresh_, NULL, 2 );
      cvDilate( foreground_thresh_, foreground_thresh_, NULL, 2 );
      //cvDilate( foreground_thresh_, foreground_thresh_, kernel_, 1 );
      cvCvtScale( foreground_thresh_, foreground_mask_, 1, 0 );
      cvZero( foreground_ );
      cvCopy( frame, foreground_, foreground_mask_ );
      //cvShowImage( "foreground", foreground_ );

      CvSeq* contour = 0;
      cvFindContours( foreground_mask_, storage_, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
      cvClearMemStorage( storage_ );

      CvRect bb;
      oit_msgs::BlobArray blobs;
      for( ; contour != 0; contour = contour->h_next )
      {

        cvDrawContours(disp_,contour,CV_RGB(0,255,0), CV_RGB(255,0,0), 0, 1, CV_AA, cvPoint(0,0));
        bb = cvBoundingRect(contour, 1 );
        oit_msgs::Blob b;
        b.x = bb.x;
        b.y = bb.y;
        b.width = bb.width;
        b.height = bb.height;
        cvRectangle( disp_, cvPoint( b.x, b.y ),
                            cvPoint( b.x+b.width, b.y+b.height ),
                            CV_RGB(255,0,0), 1 );
        blobs.blobs.push_back(b);
      }

      foreground_pub.publish( blobs );
        // undo robot contour
        // undo child contour
      */
      // publish hsv images (probably don't need this anymore)
/*
      img_.header.stamp = img_time;
      fillImage( img_, "image_rect_color",
                 frame->height, frame->width, 3, "bgr", "uint8",
                 hsv_img_->imageData );

      hsv_pub.publish( img_ );
*/      
      // TODO: fill for foreground color
      if( display > 0 )
      {
        cvShowImage( "output", disp_ );
        cvWaitKey(10);
      }
    }
    else
    {
      ROS_WARN( "img_bridge could not parse image" );
    }
  }

  void init()
  {
    //hsv_pub = n.advertise<sensor_msgs::Image>("image_hsv",1000);
    foreground_pub = n.advertise<oit_msgs::BlobArray>("foreground_blobs",1000);
    n = ros::NodeHandle("~");
    std::string colorfile, irfile;
    n.param("parent_hist", colorfile, std::string(""));
    n.param("ir_hist", irfile, std::string(""));

    hsv_img_ = cvCreateImage( cvSize( 320,240), 8, 3 );
    disp_ = cvCreateImage( cvSize( 320,240), 8, 3 );
    //foreground_img_ = cvCreateImage( cvSize( 320,240), 8, 1 );

    // colors
    color_finders_.clear();
    
    if( irfile != std::string("") )
    {
      ColorFinder c;
      c.init( irfile.c_str(), "ir" );
      color_finders_.push_back( c );
      colors_.push_back( CV_RGB( 0, 0, 255 ) );
    }


    if( colorfile != std::string( "" ) )
    {
      ColorFinder p;
      p.init( colorfile.c_str(), "child" );
      color_finders_.push_back( p );
      colors_.push_back( CV_RGB( 255, 128, 0 ) );
    }
    //std::string background_file;
    //n.param( "background", background_file, std::string("") );
    n.param( "display", display, 1 );
    if( display > 0 )
    {
      cvNamedWindow( "output", 1 );
  		//cvNamedWindow( "rectified", 1 );
      //cvNamedWindow( "foreground", 1 );
    }

/*
    printf( "background image: [%s]\n", background_file.c_str() );
    background_ = cvLoadImage(background_file.c_str() );
    foreground_ = cvCreateImage( cvGetSize( background_ ), 8, 3 );
    foreground_thresh_ = cvCreateImage(cvGetSize(background_),IPL_DEPTH_8U, 1 );
    foreground_mask_ = cvCreateImage(cvGetSize(background_ ), IPL_DEPTH_8U, 1 );
    foreground_tmp_ = cvCreateImage(cvGetSize( background_ ), IPL_DEPTH_8U, 3 );
*/
    kernel_ = cvCreateStructuringElementEx( 15, 15, 8, 8, CV_SHAPE_RECT );
    storage_ = cvCreateMemStorage(0);

		first = true;
  }

  void cleanup()
  {
    //cvReleaseImage( &hsv_img_ );
    //cvReleaseImage( &foreground_img_ );
  }
};

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"image_splitter" );
  ImageSplitter* i = new ImageSplitter();
  boost::shared_ptr<ImageSplitter> foo_object(i);
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber image_sub = it.subscribeCamera("image_raw", 1, &ImageSplitter::image_cb, foo_object );
  i->init();
  ros::spin();
  ROS_INFO( "image_splitter quitting..." );
  i->cleanup();
  return 0;
}
