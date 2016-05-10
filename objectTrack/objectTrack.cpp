////////////////////////////////////////////////////////
// RaspiVid.c was the base to develop this
// 
// Lines to command the drone added by Vítor Albiero - April 2016
// Lines added were inside "Begin Drone Code" and "End Drone Code"
// vitor_albiero@outlook.com
// 
////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

#include <wiringPi.h>

//new
#include <cv.h>
#include <highgui.h>
#include "time.h"

extern "C" {
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
}

#include <semaphore.h>

///////////////////////
// BEGIN DRONE CODE
///////////////////////

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
 
// The PiWeather board i2c address
#define ADDRESS 0x04

#define DEBUG_MODE 1 
// The I2C bus: This is for V2 pi's. For V1 Model B you need i2c-0
static const char *devName = "/dev/i2c-1";

// OPENCV
#include <iostream>
#include <fstream>
#include <sstream>
#include "time.h"

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"

#include <sys/time.h>

using namespace cv;
using namespace std;

Mat frame, cameraFeed;;
int lowThreshold = 20;

//default capture width and height
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;

///////////////////////
// END DRONE CODE
///////////////////////

/// Camera number to use - we only have one camera, indexed from 0.
#define CAMERA_NUMBER 0

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE = 30000000; // 30Mbits/s


// variable to convert I420 frame to IplImage
int nCount=0;
IplImage *py, *pu, *pv, *pu_big, *pv_big, *image,* dstImage;


int mmal_status_to_int(MMAL_STATUS_T status);

/** Structure containing all state information for the current run
*/
typedef struct
{
	int modo;
	int timeout;            /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
	int width;              /// Requested width of image
	int height;             /// requested height of image
	int bitrate;            /// Requested bitrate
	int framerate;          /// Requested frame rate (fps)
	int graymode;			/// capture in gray only (2x faster)
	int immutableInput;      /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
	/// the camera output or the encoder output (with compression artifacts)
	RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
	RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

	MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
	MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
	MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
	MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

	MMAL_POOL_T *video_pool; /// Pointer to the pool of buffers used by encoder output port

} RASPIVID_STATE;

/** Struct used to pass information in encoder port userdata to callback
*/
typedef struct
{
	FILE *file_handle;                   /// File handle to write buffer data to.
	VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
	RASPIVID_STATE *pstate;            /// pointer to our state in case required in callback
} PORT_USERDATA;

///////////////////////
// BEGIN DRONE CODE
///////////////////////

void CannyThreshold(int, void*) {

}

class Template {
	public:
		Mat img;
		string name;
};

Template templates[3];

int readRefImages(Template *templates) {
	templates[0].img = imread("/home/pi/objectTrack/takeOff.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!templates[0].img.data)
		return -1;
	threshold(templates[0].img, templates[0].img, 100, 255, 0);
	templates[0].name = "TakeOff";

	templates[1].img = imread("/home/pi/objectTrack/land.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!templates[1].img.data)
		return -1;
	threshold(templates[1].img, templates[1].img, 100, 255, 0);
	templates[1].name = "Land";
	
	templates[2].img = imread("/home/pi/objectTrack/follow.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!templates[2].img.data)
		return -1;
	threshold(templates[2].img, templates[2].img, 100, 255, 0);
	templates[2].name = "Follow";

	
	return 0;
}

void sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center) {
	std::vector<cv::Point2f> top, bot;

	for (unsigned int i = 0; i < corners.size(); i++) {
		if (corners[i].y < center.y)
			top.push_back(corners[i]);
		else
			bot.push_back(corners[i]);
	}

	cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
	cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
	cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
	cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

	corners.clear();
	corners.push_back(tl);
	corners.push_back(tr);
	corners.push_back(br);
	corners.push_back(bl);
}

string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}


void sendI2C(int i2cX, int i2cY, int i2cZ, int i2cMode)
{
  //printf("I2C: Connecting\n");
  int file;
 
  if ((file = open(devName, O_RDWR)) < 0) {
    fprintf(stderr, "I2C: Failed to access %d\n", devName);
    exit(1);
  }
 
  //printf("I2C: acquiring buss to 0x%x\n", ADDRESS);
 
  if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) {
    fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);
    exit(1);
  }
  
  int val;
  unsigned char cmd[32]; 
 
  //printf("Sending %d\n", i2cX);
  //printf("Sending %d\n", i2cY);
	
	cmd[0] = (i2cX >> 8) & 0xFF;
	cmd[1] = i2cX & 0xFF;
	cmd[2] = i2cY;
	cmd[3] = i2cZ;
	cmd[4] = i2cMode;
	
	
    if (write(file, cmd, 5) == 5) {
      //usleep(10000);
    }
	
  close(file);
}

void gpio(int Ama, int Verm, int Verd){
	digitalWrite (29, Ama);
	digitalWrite (28, Verm);
	digitalWrite (27, Verd);
}

int readI2C()
{
  //printf("I2C: Connecting\n");
  int file;

  if ((file = open(devName, O_RDWR)) < 0) {
    fprintf(stderr, "I2C: Failed to access %d\n", devName);
    exit(1);
  }
 
  //printf("I2C: acquiring buss to 0x%x\n", ADDRESS);
 
  if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) {
    fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);
    exit(1);
  }
  
  unsigned char cmd[16]; 
  int temp;
  if (read(file, cmd, 1) == 1) {
	temp = cmd[0];
  }
  //usleep(10000);	
  close(file);
  return temp;
}

void drawObject(int x, int y,Mat &frame){
	if(y-5>0)
		line(frame,Point(x,y),Point(x,y-5),Scalar(180),2);
	else line(frame,Point(x,y),Point(x,0),Scalar(180),2);
	if(y+5<FRAME_HEIGHT)
		line(frame,Point(x,y),Point(x,y+5),Scalar(180),2);
	else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(180),2);
	if(x-5>0)
		line(frame,Point(x,y),Point(x-5,y),Scalar(180),2);
	else line(frame,Point(x,y),Point(0,y),Scalar(180),2);
	if(x+5<FRAME_WIDTH)
		line(frame,Point(x,y),Point(x+5,y),Scalar(180),2);
	else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(180),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(180),2);

}

void trackObject(Mat &camera){
	int OutputX;
	int OutputY;
	int OutputZ;
	int instMode;
		
	Mat greyImg;
	
	camera.copyTo(greyImg);
	
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	GaussianBlur(greyImg, greyImg, Size(9, 9), 2, 2);

	/// Detect edges using canny
	Canny(greyImg, canny_output, lowThreshold, lowThreshold * 3, 3);

	imshow("B",canny_output);
	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<Point> approxRect;
	
	for (size_t i = 0; i < contours.size(); i++) {
		
		approxPolyDP(contours[i], approxRect, arcLength(Mat(contours[i]), true) * 0.05, true);
		
		if (approxRect.size() == 4) {
			float area = contourArea(contours[i]);

			if (area > 6000) {
				std::vector<cv::Point2f> corners;

				vector<Point>::iterator vertex;
				vertex = approxRect.begin();
				//vertex++;
				circle(camera, *vertex, 4, Scalar(180), -1, 8, 0);
				corners.push_back(*vertex);
				vertex++;
				circle(camera, *vertex, 4, Scalar(180), -1, 8, 0);
				corners.push_back(*vertex);
				vertex++;
				circle(camera, *vertex, 4, Scalar(180), -1, 8, 0);
				corners.push_back(*vertex);
				vertex++;
				circle(camera, *vertex, 4, Scalar(180), -1, 8, 0);
				corners.push_back(*vertex);

				Moments mu;
				mu = moments(contours[i], false);
				Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);

				sortCorners(corners, center);

				// Define the destination image
				Mat correctedImg = ::Mat::zeros(205, 285, CV_8UC3);

				// Corners of the destination image
				std::vector<cv::Point2f> quad_pts;
				quad_pts.push_back(Point2f(0, 0));
				quad_pts.push_back(Point2f(correctedImg.cols, 0));
				quad_pts.push_back(Point2f(correctedImg.cols, correctedImg.rows));
				quad_pts.push_back(Point2f(0, correctedImg.rows));

				// Get transformation matrix
				Mat transmtx = getPerspectiveTransform(corners, quad_pts);

				// Apply perspective transformation
				warpPerspective(greyImg, correctedImg, transmtx, correctedImg.size());

				//Mat correctedImgBin;
				Mat new_image;

				correctedImg.copyTo(new_image);
				
				double minVal,maxVal,medVal;

				minMaxLoc(new_image, &minVal, &maxVal);

				medVal=(maxVal-minVal)/2;

				threshold(new_image, new_image, medVal, 255, 0);

				if (DEBUG_MODE == 1)
					imshow("C", new_image);

				Mat diffImg;

				int match, minDiff, diff;
				minDiff = 5000;
				match = -1;

				
				for (int i = 0; i < 3; i++) {
					bitwise_xor(new_image, templates[i].img, diffImg, noArray());
					
					diff = countNonZero(diffImg);
					if (diff < minDiff) {
						minDiff = diff;
						match = i;
					}
					
				}
				
				switch (match)
				{
					case 0: //take off
						sendI2C(0, 0, 0, match+1); // send the data to arduino
						gpio(0, 0, 1); // turn on a green led
						break;
						
					case 1: // land
						sendI2C(0, 0, 0, match+1); 
						gpio(0, 0, 1);
						break;
						
					case 2: //follow object
						OutputX = mu.m10 / area; 
						OutputY = mu.m01 / area;
						OutputZ = (255 / area) * 5000;
						if (DEBUG_MODE == 1)
							drawObject(OutputX, OutputY, camera);
						sendI2C(OutputX, OutputY, OutputZ, match+1); 
						gpio(0, 0, 1);
						break;
				
					default:
						sendI2C(0,0,0,0); 
						gpio(0, 1, 0); //turn on a red led
				}
				if (DEBUG_MODE == 1) {
					if (match != -1) {
						putText(camera, templates[match].name, Point(100, 20), 1,
								2, Scalar(180), 2);
						imshow("D", templates[match].img);
					}
					
				}
			}
		}
		else {
			sendI2C(0,0,0,0); 
			gpio(0, 1, 0); //turn on a red led
			
		}
	} 	
}

///////////////////////
// END DRONE CODE
///////////////////////

// default status
static void default_status(RASPIVID_STATE *state)
{
	if (!state)
	{
		vcos_assert(0);
		return;
	}

	// Default everything to zero
	memset(state, 0, sizeof(RASPIVID_STATE));

	// Now set anything non-zero
	state->timeout 			= 300000000;     // capture time : here 65 s
	state->width 			= FRAME_WIDTH;      	 // use a multiple of 320 (640, 1280)
	state->height 			= FRAME_HEIGHT;		     // use a multiple of 240 (480, 960)
	state->bitrate 			= 17000000;      // This is a decent default bitrate for 1080p
	state->framerate 		= VIDEO_FRAME_RATE_NUM;
	state->immutableInput 	= 1;
	state->graymode 		= 1;			//gray (1), much faster than color (0)

	// Setup preview window defaults
	raspipreview_set_defaults(&state->preview_parameters);

	// Set up the camera_parameters to default
	raspicamcontrol_set_defaults(&state->camera_parameters);
}

/**
*  buffer header callback function for video
*
* @param port Pointer to port from which callback originated
* @param buffer mmal buffer header pointer
*/
static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	MMAL_BUFFER_HEADER_T *new_buffer;
	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

	if (pData)
	{

		if (buffer->length)
		{

			mmal_buffer_header_mem_lock(buffer);

			///////////////////////
			// BEGIN DRONE CODE
			///////////////////////
			int w=pData->pstate->width;	// get image size
			int h=pData->pstate->height;
			int h4=h/4;

			memcpy(py->imageData,buffer->data,w*h);	// read Y

			if (pData->pstate->graymode==0)
			{
				memcpy(pu->imageData,buffer->data+w*h,w*h4); // read U
				memcpy(pv->imageData,buffer->data+w*h+w*h4,w*h4); // read v

				cvResize(pu, pu_big, CV_INTER_NN);
				cvResize(pv, pv_big, CV_INTER_NN);  //CV_INTER_LINEAR looks better but it's slower
				cvMerge(py, pu_big, pv_big, NULL, image);

				cvCvtColor(image,dstImage,CV_YCrCb2RGB);	// convert in RGB color space (slow)
				cameraFeed=cvarrToMat(dstImage);   
				//cvShowImage("A", dstImage );
				//cameraFeed=cvarrToMat(py); 

			}
			else
			{	
				cameraFeed=cvarrToMat(py);  
				//cvShowImage("A", py); // display only gray channel
			}
			
			
			int readRasp = readI2C();
			//
			if (readRasp != 1)
				gpio(1, 0, 0); //turn on a yellow led
			
				
			if (readRasp == 1){
				trackObject(cameraFeed);

				//delay 30ms so that screen can refresh.
				//waitKey(30);
				
			} else if (readRasp == 8){
				exit(0);
			} else if (readRasp == 9){
				system ("sudo shutdown -h now");
			}
						
			if (DEBUG_MODE == 1)
				imshow("A", cameraFeed);
			waitKey(1);
			
			///////////////////////
			// END DRONE CODE
			///////////////////////
			nCount++;		// count frames displayed

			mmal_buffer_header_mem_unlock(buffer);
		}
		else vcos_log_error("buffer null");

	}
	else
	{
		vcos_log_error("Received a encoder buffer callback with no state");
	}

	// release buffer back to the pool
	mmal_buffer_header_release(buffer);

	// and send one back to the port (if still open)
	if (port->is_enabled)
	{
		MMAL_STATUS_T status;

		new_buffer = mmal_queue_get(pData->pstate->video_pool->queue);

		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);

		if (!new_buffer || status != MMAL_SUCCESS)
			vcos_log_error("Unable to return a buffer to the encoder port");
	}
}


/**
* Create the camera component, set up its ports
*
* @param state Pointer to state control struct
*
* @return 0 if failed, pointer to component if successful
*
*/
static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state)
{
	MMAL_COMPONENT_T *camera = 0;
	MMAL_ES_FORMAT_T *format;
	MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
	MMAL_STATUS_T status;
	//cria componente camera
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create camera component");
		goto error;
	}

	if (!camera->output_num)
	{
		vcos_log_error("Camera sem porta de saída");
		goto error;
	}

	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

	{	//  configure the camera
		MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
		{
			{ MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
			cam_config.max_stills_w = state->width,
			cam_config.max_stills_h = state->height,
			cam_config.stills_yuv422 = 0,
			cam_config.one_shot_stills = 0,
			cam_config.max_preview_video_w = state->width,
			cam_config.max_preview_video_h = state->height,
			cam_config.num_preview_video_frames = 3,
			cam_config.stills_capture_circular_buffer_height = 0,
			cam_config.fast_preview_resume = 0,
			cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
		};
		mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}
	// set the encoders types to the camera
	format = video_port->format;
	format->encoding_variant = MMAL_ENCODING_I420;
	format->encoding = MMAL_ENCODING_I420;
	format->es->video.width = state->width;
	format->es->video.height = state->height;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state->width;
	format->es->video.crop.height = state->height;
	format->es->video.frame_rate.num = state->framerate;
	format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(video_port);
	if (status)
	{
		vcos_log_error("camera video format couldn't be set");
		goto error;
	}

	// plug the callback to the video port 
	status = mmal_port_enable(video_port, video_buffer_callback);
	if (status)
	{
		vcos_log_error("camera video callback2 error");
		goto error;
	}

	// Ensure there are enough buffers to avoid dropping frames
	if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


	// Set the encode format on the still  port
	format = still_port->format;
	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;
	format->es->video.width = state->width;
	format->es->video.height = state->height;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state->width;
	format->es->video.crop.height = state->height;
	format->es->video.frame_rate.num = 1;
	format->es->video.frame_rate.den = 1;

	status = mmal_port_format_commit(still_port);
	if (status)
	{
		vcos_log_error("camera still format couldn't be set");
		goto error;
	}


	// create pool of message on video port
	MMAL_POOL_T *pool;
	video_port->buffer_size = video_port->buffer_size_recommended;
	video_port->buffer_num = video_port->buffer_num_recommended;
	pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
	if (!pool)
	{
		vcos_log_error("Failed to create buffer header pool for video output port");
	}
	state->video_pool = pool;

	/* Ensure there are enough buffers to avoid dropping frames */
	if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

	/* Enable component */
	status = mmal_component_enable(camera);

	if (status)
	{
		vcos_log_error("camera component couldn't be enabled");
		goto error;
	}

	raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

	state->camera_component = camera;

	return camera;

error:

	if (camera)
		mmal_component_destroy(camera);

	return 0;
}

/**
* Destroy the camera component
*
* @param state Pointer to state control struct
*
*/
static void destroy_camera_component(RASPIVID_STATE *state)
{
	if (state->camera_component)
	{
		mmal_component_destroy(state->camera_component);
		state->camera_component = NULL;
	}
}


/**
* Destroy the encoder component
*
* @param state Pointer to state control struct
*
*/
static void destroy_encoder_component(RASPIVID_STATE *state)
{
	// Get rid of any port buffers first
	if (state->video_pool)
	{
		mmal_port_pool_destroy(state->encoder_component->output[0], state->video_pool);
	}

	if (state->encoder_component)
	{
		mmal_component_destroy(state->encoder_component);
		state->encoder_component = NULL;
	}
}

/**
* Connect two specific ports together
*
* @param output_port Pointer the output port
* @param input_port Pointer the input port
* @param Pointer to a mmal connection pointer, reassigned if function successful
* @return Returns a MMAL_STATUS_T giving result of operation
*
*/
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
	MMAL_STATUS_T status;

	status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

	if (status == MMAL_SUCCESS)
	{
		status =  mmal_connection_enable(*connection);
		if (status != MMAL_SUCCESS)
			mmal_connection_destroy(*connection);
	}

	return status;
}

/**
* Checks if specified port is valid and enabled, then disables it
*
* @param port  Pointer the port
*
*/
static void check_disable_port(MMAL_PORT_T *port)
{
	if (port && port->is_enabled)
		mmal_port_disable(port);
}

/**
* Handler for sigint signals
*
* @param signal_number ID of incoming signal.
*
*/
static void signal_handler(int signal_number)
{
	// Going to abort on all signals
	vcos_log_error("Aborting program\n");

	// TODO : Need to close any open stuff...how?

	exit(255);
}

/**
* main
*/

int main(int argc, const char **argv)
{	
	if (DEBUG_MODE == 1){
		namedWindow("A", CV_WINDOW_AUTOSIZE);
		namedWindow("B", CV_WINDOW_AUTOSIZE);
		namedWindow("C", CV_WINDOW_AUTOSIZE);
		namedWindow("D", CV_WINDOW_AUTOSIZE);
	}
		
	wiringPiSetup ();
	pinMode(29, OUTPUT);
	pinMode(28, OUTPUT);
	pinMode(27, OUTPUT);
	
	if (readRefImages(templates) == -1) {
		printf("Error reading reference templates\n");
		return -1;
	}
	if (DEBUG_MODE == 1)
		createTrackbar("Min Threshold:", "A", &lowThreshold, 100, CannyThreshold);
	
	// Our main data storage vessel..
	RASPIVID_STATE state; //CAPTURA CAMERA

	MMAL_STATUS_T status;// = -1;
	MMAL_PORT_T *camera_video_port = NULL;
	MMAL_PORT_T *camera_still_port = NULL;
	MMAL_PORT_T *preview_input_port = NULL;
	MMAL_PORT_T *encoder_input_port = NULL;
	MMAL_PORT_T *encoder_output_port = NULL;

	time_t timer_begin,timer_end;
	double secondsElapsed;

	bcm_host_init();
	signal(SIGINT, signal_handler);

	// read default status
	default_status(&state);

	// init windows and OpenCV Stuff
	//cvNamedWindow("A", CV_WINDOW_AUTOSIZE); 
	int w=state.width;
	int h=state.height;
	dstImage = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
	py = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);		// Y component of YUV I420 frame
	pu = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);	// U component of YUV I420 frame
	pv = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);	// V component of YUV I420 frame
	pu_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
	pv_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
	image = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);	// final picture to display
	
	// create camera
	if (!create_camera_component(&state))
	{
		vcos_log_error("%s: Failed to create camera component", __func__);
	}
	else if ((status=raspipreview_create(&state.preview_parameters))!=MMAL_SUCCESS)
	{
		vcos_log_error("%s: Failed to create preview component", __func__);
		destroy_camera_component(&state);
	}
	else
	{
		PORT_USERDATA callback_data;

		camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
		camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];

		VCOS_STATUS_T vcos_status;

		callback_data.pstate = &state;

		vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
		vcos_assert(vcos_status == VCOS_SUCCESS);

		// assign data to use for callback
		camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;

		// init timer
		time(&timer_begin); 


		// start capture
		if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
		{
			return 0;
		}

		// Send all the buffers to the video port

		int num = mmal_queue_length(state.video_pool->queue);
		int q;
		for (q=0;q<num;q++)
		{
			MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.video_pool->queue);

			if (!buffer)
				vcos_log_error("Unable to get a required buffer %d from pool queue", q);

			if (mmal_port_send_buffer(camera_video_port, buffer)!= MMAL_SUCCESS)
				vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
		}

		// Now wait until we need to stop
		vcos_sleep(state.timeout);

		//mmal_status_to_int(status);
		// Disable all our ports that are not handled by connections
		check_disable_port(camera_still_port);

		if (state.camera_component)
			mmal_component_disable(state.camera_component);

		//destroy_encoder_component(&state);
		raspipreview_destroy(&state.preview_parameters);
		destroy_camera_component(&state);

	}
	if (status != 0)
		raspicamcontrol_check_configuration(128);

	time(&timer_end);  /* get current time; same as: timer = time(NULL)  */

	cvReleaseImage(&dstImage);
	cvReleaseImage(&pu);
	cvReleaseImage(&pv);
	cvReleaseImage(&py);
	cvReleaseImage(&pu_big);
	cvReleaseImage(&pv_big);
	
	secondsElapsed = difftime(timer_end,timer_begin);

	printf ("%.f seconds for %d frames : FPS = %f\n", secondsElapsed,nCount,(float)((float)(nCount)/secondsElapsed));
	
	return 0;	
}

