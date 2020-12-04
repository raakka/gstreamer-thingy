#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <netdb.h>

#include "helper.hpp"
#include "cam_helper.hpp"
#include "gst_pipeline.hpp"
#include "color.hpp"
#include "TestPipeline.h"

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

using namespace std;
using namespace cv;

#define NANO___

#define DS_INFO_VERSION 1
#define BUFLEN 512
#define PORT 5810

char srv_ip[] = "192.168.1.76";

char                     *target_rov;
int                       i, k;
unsigned int              frame_cnt;
unsigned int              bit0, bit1, bit2, bit3, bit4, bit5, bit6, bit7, bit8, bit9, bit10;
unsigned int              udp_frame_timer;
unsigned int              display_update_timer;
struct sockaddr_in        local, remote;
int                       sockfd, remote_len;
char                      buf[BUFLEN];
//char                      xbuf[BUFLEN];
int                       len, retval;
int                       rx_packet_count, tx_packet_count;
int                       nonBlocking = 1;
int                       optval;
int                       optlen;
char                     *optval2;
time_t                    current_time, prev_time;
uint                      rx_count = 0;
uint                      tx_count = 0;

struct vision_tmp_t
{
    int version; // packet version
    int valid;   // status of data
    int who;
    
    // TODO(JACOB): we only need the x and y
    // change the packet to only contain LRXY
    int Lx;
    int Ly;
    int Lz;

    int Rx;
    int Ry;
    int Rz;

    int blur;
    int hue_min;
    int hue_max;
    int sat_min;
    int sat_max;
    int lv_min;
    int lv_max;
    int db_sync_request;
    int db_store;
    int db_sync_flag;
    int pad;
};

struct vision_tmp_t vision_tmp, vision_db;

////////////////////////////////////////////////////////////////////////////////
void get_config()
{
    FILE* vconfig; 
 
    vconfig = fopen("vision_config.txt", "r+"); 
    if (vconfig == NULL )   
    {
    	vconfig = fopen("vision_config.txt", "w+"); 

	fprintf(vconfig, "%d, %d, %d, %d, %d, %d, %d\n", 
	    vision_tmp.hue_min,
	    vision_tmp.hue_max,
	    vision_tmp.sat_min,
	    vision_tmp.sat_max,
	    vision_tmp.lv_min,
	    vision_tmp.lv_max,
	    vision_tmp.blur);

	fclose(vconfig);
    }

    vconfig = fopen("vision_config.txt", "r+"); 
    if (vconfig != NULL )   
    {
	rewind(vconfig);

	fscanf(vconfig, "%d, %d, %d, %d, %d, %d, %d\n", 
	    &vision_tmp.hue_min,
	    &vision_tmp.hue_max,
	    &vision_tmp.sat_min,
	    &vision_tmp.sat_max,
	    &vision_tmp.lv_min,
	    &vision_tmp.lv_max,
	    &vision_tmp.blur);

	fclose(vconfig);
    }
    printf("%d, %d, %d, %d, %d, %d, %d\n", 
        vision_tmp.hue_min,
        vision_tmp.hue_max,
        vision_tmp.sat_min,
        vision_tmp.sat_max,
        vision_tmp.lv_min,
        vision_tmp.lv_max,
        vision_tmp.blur);
}

////////////////////////////////////////////////////////////////////////////////
void save_config()
{
    FILE* vconfig; 
    
    vconfig = fopen("vision_config.txt", "w"); 
    if (vconfig != NULL )   
    {
	rewind(vconfig);

	fprintf(vconfig, "%d, %d, %d, %d, %d, %d, %d\n", 
	    vision_tmp.hue_min,
	    vision_tmp.hue_max,
	    vision_tmp.sat_min,
	    vision_tmp.sat_max,
	    vision_tmp.lv_min,
	    vision_tmp.lv_max,
	    vision_tmp.blur);

	fclose(vconfig);
    }
}

////////////////////////////////////////////////////////////////////////////////
int init_udp(void)
{
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        perror("socket sockfd");
        _exit(1);
    }

    memset((char *) &local, 0, sizeof (local));
    local.sin_family = AF_INET;
    local.sin_port = htons(PORT);
    local.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sockfd, (const struct sockaddr *) &local, sizeof (local)) == -1)
    {
        perror("bind");
        _exit(1);
    }

    fcntl(sockfd, F_SETFL, O_NONBLOCK, nonBlocking);

    // set SO_REUSEADDR on a socket to true (1):
    optval = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

    // see if the SO_BROADCAST flag is set:
    // getsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &optval, &optlen);
    // if (optval != 0)
    // {
    // printf("SO_BROADCAST enabled on sockfd!\n");
    // }
}

////////////////////////////////////////////////////////////////////////////////
int process_rx_udp(void)
{
    int ixx;

    for (ixx = 0 ; ixx < 10 ; ixx++)
    {
        remote_len = sizeof (remote);
        retval = recvfrom(sockfd, buf, BUFLEN, 0, (struct sockaddr *) &remote, (socklen_t*) &remote_len);
        if (retval > 0)
        {
            rx_count++;

            vision_tmp.version = buf[0];
            vision_tmp.valid = buf[1];  
            vision_tmp.who = buf[2];

            vision_tmp.Lx = buf[3];
            vision_tmp.Ly = buf[4];
            vision_tmp.Lz = buf[5];

            vision_tmp.Rx = buf[6];
            vision_tmp.Ry = buf[7];
            vision_tmp.Rz = buf[8];

            vision_tmp.blur = buf[9];
            vision_tmp.hue_min = buf[10];
            vision_tmp.hue_max = buf[11];
            vision_tmp.sat_min = buf[12];
            vision_tmp.sat_max = buf[13];
            vision_tmp.lv_min = buf[14];
            vision_tmp.lv_max = buf[15];
            vision_tmp.db_sync_request = buf[16];
            vision_tmp.db_store = buf[17];
            vision_tmp.db_sync_flag = buf[18];
            vision_tmp.pad = buf[19];

            if (vision_tmp.db_sync_request != 0)
                memcpy(&vision_tmp, &vision_db, sizeof(struct vision_tmp_t) - 2);

            if (vision_tmp.db_store == 1)
            {
                memcpy(&vision_db, &vision_tmp, sizeof(struct vision_tmp_t) - 2);
                save_config();
            }
        }
        goto done;
    }
    done: ;
}

////////////////////////////////////////////////////////////////////////////////
int process_tx_udp(void)
{
    memset((char *) &remote, 0, sizeof (remote));
    remote.sin_family = AF_INET;
    remote.sin_port = htons(PORT);
    remote.sin_addr.s_addr = inet_addr(target_rov);

    remote_len = sizeof (remote);

    ////////////////////

    buf[0] = vision_tmp.version;
    buf[1] = vision_tmp.valid;  
    buf[2] = vision_tmp.who;

    buf[3] = vision_tmp.Lx;
    buf[4] = vision_tmp.Ly;
    buf[5] = vision_tmp.Lz;

    buf[6] = vision_tmp.Rx;
    buf[7] = vision_tmp.Ry;
    buf[8] = vision_tmp.Rz;

    buf[9] = vision_tmp.blur;
    buf[10] = vision_tmp.hue_min;
    buf[11] = vision_tmp.hue_max;
    buf[12] = vision_tmp.sat_min;
    buf[13] = vision_tmp.sat_max;
    buf[14] = vision_tmp.lv_min;
    buf[15] = vision_tmp.lv_max;
    buf[16] = vision_tmp.db_sync_request;
    buf[17] = vision_tmp.db_store;
    buf[18] = vision_tmp.db_sync_flag;
    buf[19] = vision_tmp.pad;

    ////////////////////

    len = sizeof(vision_tmp);	

    tx_count++;
    if (sendto(sockfd, buf, len, 0, (const struct sockaddr *) &remote, remote_len) == -1)
        perror("sendto");

}

////////////////////////////////////////////////////////////////////////////////
int process_display()
{
    printf("%c[%d;%dH", 27, 3, 5);
    printf("Hue Min : %4d : %4d", vision_tmp.hue_min, vision_db.hue_min);
    printf("%c[%d;%dH", 27, 3, 45);
    printf("Hue Max : %4d : %4d", vision_tmp.hue_max, vision_db.hue_max);

    printf("%c[%d;%dH", 27, 5, 5);
    printf("Sat Min : %4d : %4d", vision_tmp.sat_min, vision_db.sat_min);
    printf("%c[%d;%dH", 27, 5, 45);
    printf("Sat Max : %4d : %4d", vision_tmp.sat_max, vision_db.sat_max);

    printf("%c[%d;%dH", 27, 7, 5);
    printf("L/V Min : %4d : %4d", vision_tmp.lv_min, vision_db.lv_min);
    printf("%c[%d;%dH", 27, 7, 45);
    printf("L/V Max : %4d : %4d", vision_tmp.lv_max, vision_db.lv_max);

    printf("%c[%d;%dH", 27, 9, 45);
    printf("Blur    : %4d : %4d", vision_tmp.blur, vision_db.blur);

    printf("%c[%d;%dH", 27, 13, 5);
    printf("Database Sync    : %4d", vision_tmp.db_sync_flag);

    printf("%c[%d;%dH", 27, 15, 5);
    printf("RX        : %4d", rx_count);

    printf("%c[%d;%dH", 27, 15, 25);
    printf("TX        : %4d", tx_count);

    printf("%c[%d;%dH", 27, 17, 0);

    fflush(stdout);
}

////////////////////////////////////////////////////////////////////////////////

#ifdef NANO
    std::string get_tegra_pipeline(int width, int height, int fps) {
        return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" +
            std::to_string(height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(fps) +
            "/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }
#endif

// shared_ptr<NetworkTable> myNetworkTable; //our networktable for reading/writing
// string netTableAddress = "192.168.1.34"; //address of the rio

//camera parameters
int 
device = 0, //bc we are using video0 in this case
width = 640, 
height = 360, 
framerate = 30, 
mjpeg = false; //mjpeg is not better than just grabbing a raw image in this case

//network parameters: change this according to your specific network configuration
int
bitrate = 8000000, // kbit/sec over network
port_stream = 5803, //destination port for raw image
port_thresh = 5805; //destination port for thresholded image
//string ip = "10.13.11.20"; //destination ip
string ip = "192.168.1.76"; //destination ip

string tableName = "CVResultsTable";
bool verbose = true;

namespace params {
    int test_x = 50, test_y = 50;
    int min_hue = 0, max_hue = 255;
    int min_sat = 0, max_sat = 255;
    int min_val = 0, max_val = 255;
    int blur = 5;
}

CameraSettings cam_settings;

struct ImgResults {
    int hue, sat, val;
};

ImgResults simple_pipeline (const cv::Mat &bgr, cv::Mat &processedImage);
void send_initial_img_params ();
void update_img_params ();
void set_cam_params ();
void update_camera_params ();

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{

    int loop_counter = 0;
    int retval;
    char configstring[80];

    //grip::GripPipeline m_grip;
    grip::TestPipeline m_grip;
    cv::Mat m_cameraFrame;
    cv::Mat m_tmp;
    cv::Mat m_tmp2;
    cv::Mat m_tmp3;
    cv::Mat *m_blur;
    cv::Mat *m_thresh;
    cv::Mat m_gripFrame;

    std::vector<cv::KeyPoint> *m_blob;

    // IplImage transmit_image;

    double hsvThresholdHue[] = {33.0, 90.0};
    double hsvThresholdSaturation[] = {0.0, 245.0};
    double hsvThresholdValue[] = {84.0, 235.0};

    if (argc > 1)
        target_rov = argv[1];
    else
        target_rov = srv_ip;

    get_config();
    init_udp();

    system("clear");
    printf("%c[%d;%dH", 27, 1, 5);
    printf("Vision Processor");
    printf("%c[%d;%dH", 27, 1, 25);
    printf("Created by Kell Robotics");
    printf("%c[%d;%dH", 27, 22, 5);
    printf("Remote IP : %s", target_rov);

    memcpy(&vision_db, &vision_tmp, sizeof(struct vision_tmp_t));

    /////////////////////////////////////////////////////////////////////

#ifdef NANO
    // Options
    int WIDTH = 640;
    int HEIGHT = 360;
    int FPS = 30;

    // Define the gstream pipeline
    std::string pipeline = get_tegra_pipeline(WIDTH, HEIGHT, FPS);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    // Create OpenCV capture object, ensure it works.
    cv::VideoCapture camera(pipeline, cv::CAP_GSTREAMER);
    if (!camera.isOpened()) {
        std::cout << "Connection failed";
        return -1;
    }
#else
   // cv::VideoCapture camera(0, cv::CAP_V4L2);
   cv::VideoCapture camera(0, cv::CAP_GSTREAMER);

   if (!camera.isOpened()) {
       std::cout << "Connection failed";
       return -1;
   }
   
   int camera_fps = 30;
   int camera_width = 640;
   int camera_height = 360;

   // int camera_codec = CV_FOURCC('M','J','P','G');
   // camera.set(CV_CAP_PROP_FOURCC, camera_codec);
   camera.set(CAP_PROP_FPS, camera_fps);
   camera.set(CAP_PROP_FRAME_WIDTH, camera_width);
   camera.set(CAP_PROP_FRAME_HEIGHT, camera_height);
#endif

/*
    int fourcc = VideoWriter::fourcc('M','J','P','G');
    // int fourcc = VideoWriter::fourcc('X','2','6','4');

   VideoWriter videostream("appsrc ! video/x-raw, format=(string)BGR, width=(int)640, height=(int)480, framerate=(fraction)30/1 ! videoconvert ! omxh264enc bitrate=8000000 ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay ! udpsink host=10.13.11.20 port=5805",
       CAP_GSTREAMER,
       fourcc,              // fourcc 
       30,             // fps
       Size(1280,720),
       true);  // isColor,
*/
/*
 int fourcc = VideoWriter::fourcc('X','2','6','4');
    VideoWriter videostream(
    // "appsrc ! nvvidconv ! nvv4l2h264enc ! video/x-h264,stream-format=byte-stream, width=640, height=360, framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=$CLIENT port=5805 ",
       "appsrc ! nvvidconv ! omxh264enc ! video/x-h264,stream-format=byte-stream, width=640, height=360, framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=$CLIENT port=5805 ",
        CAP_GSTREAMER,
        fourcc,              // fourcc 
        30,             // fps
        Size(640, 360),
        true);  // isColor,
*/

    namedWindow("orig");
    moveWindow("orig", 20,20);

    namedWindow("blur");
    moveWindow("blur", 720,20);

    namedWindow("thresh");
    moveWindow("thresh", 1420,20);

    CvVideoWriter_GStreamer mywriter;
    string write_pipeline = create_write_pipeline (width, height, framerate, bitrate, ip, port_thresh);
    if (verbose) {
        printf ("GStreamer write pipeline: %s\n", write_pipeline.c_str());
    }
    mywriter.open (write_pipeline.c_str(), 0, framerate, cv::Size(width, height), true);

    cv::Mat img(640,480,CV_8UC3);

    while (1) 
    {
        /////////////////////////////////////////////////////////////////////

        process_rx_udp();

        if ( (vision_db.hue_min == vision_tmp.hue_min)
        && (vision_db.hue_max == vision_tmp.hue_max)
        && (vision_db.sat_min == vision_tmp.sat_min)
        && (vision_db.sat_max == vision_tmp.sat_max)
        && (vision_db.lv_min == vision_tmp.lv_min)
        && (vision_db.lv_max == vision_tmp.lv_max)
        && (vision_db.blur == vision_tmp.blur) 
        )
            vision_tmp.db_sync_flag = 1;
        else
            vision_tmp.db_sync_flag = 0;
            
        // process_display();       
        process_tx_udp();

        loop_counter++;	
        if (loop_counter > 10) 
	    {
            loop_counter = 0;	
            process_display();
    	}

        /////////////////////////////////////////////////////////////////////

        m_grip.hsvThresholdHue[0] = vision_tmp.hue_min;
        m_grip.hsvThresholdHue[1] = vision_tmp.hue_max;
        
        m_grip.hsvThresholdSaturation[0] = vision_tmp.sat_min;
        m_grip.hsvThresholdSaturation[1] = vision_tmp.sat_max;
        
        m_grip.hsvThresholdValue[0] = vision_tmp.lv_min;
        m_grip.hsvThresholdValue[1] = vision_tmp.lv_max;

        m_grip.blurRadius = vision_tmp.blur;

        /////////////////////////////////////////////////////////////////////

        camera.read(m_cameraFrame);
        m_cameraFrame.convertTo(m_gripFrame, CV_8UC3);
        imshow("orig", m_gripFrame);

        m_grip.Process(m_gripFrame);

        m_blur = m_grip.GetBlurOutput();
        imshow("blur", *m_blur);
        
        m_thresh = m_grip.GetHsvThresholdOutput();
        imshow("thresh", *m_thresh);

        img = m_cameraFrame;

        IplImage transmit_image = (IplImage) img;
        // IplImage transmit_image = (IplImage) *m_blur;
        // IplImage transmit_image = (IplImage) *m_thresh;

        mywriter.writeFrame (&transmit_image); //write output image over network

        // usleep(10);
        cv::waitKey(30); //needed to show frame
    }

    close(sockfd);
    return 0;
}
