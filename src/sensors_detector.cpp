#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>

using namespace cv;
using namespace std;

// Limiares da cor vermelha ( Imagem BGR )
#define MAXRED 25
#define MINRED 0

#define MAXSATRED 45
#define MINSATRED 0

#define MAXVALRED 165
#define MINVALRED 105


// Limiares da cor laranja ( Imagem HSV )
#define MINORANGE 5
#define MAXORANGE 25

#define MINSATORANGE 45
#define MAXSATORANGE 80

#define MINVALORANGE 90
#define MAXVALORANGE 155

#define cvCOLOR_RED Scalar(0, 0, 255)


int ARR_MINORANGE[3] = {MINORANGE, MINSATORANGE, MINVALORANGE};
int ARR_MAXORANGE[3] = {MAXORANGE, MAXSATORANGE, MAXVALORANGE};

int ARR_MINRED[3] = {MINRED, MINSATRED, MINVALRED};
int ARR_MAXRED[3] = {MAXRED, MAXSATRED, MAXVALRED};

//parametros de filtros
#define GAUSSIANFILTER 3
#define KERNELSIZE 7


class FindSensor
{
public:

    Mat mainImage_C3, imageHSV_C3, image_C1;
    Mat kernel;
    Mat tubo, marcador_ruim;

    Rect badmark;

    int centerX, centerY;

    // Variaveis do findContours
    vector<vector<Point>> contours;

    std_msgs::Bool found_sensor;

    FindSensor()
    {
        this->kernel = Mat::ones(KERNELSIZE, KERNELSIZE, CV_8U);
    }


    void camParam(Mat img)
    {
        this->centerX = img.rows / 2;
        this->centerY = img.cols / 2;
    }


    void setImage(Mat img)
    {
        this->camParam(img);

        this->mainImage_C3 = img;
    }


    void processImage()
    {
        Mat bitwise_tubo, bitwise_badmark;

        GaussianBlur(this->mainImage_C3, this->mainImage_C3, Size(GAUSSIANFILTER, GAUSSIANFILTER), 0);

        // Converte de BGR para HSV (MAL DESEMPENHO)
        // cvtColor(this->mainImage_C3, this->imageHSV_C3, COLOR_BGR2HSV);


        this->tubo = imlimiares(this->mainImage_C3, ARR_MINORANGE, ARR_MAXORANGE);
        bitwise_and(mainImage_C3, mainImage_C3, bitwise_tubo, this->tubo);

		morphologyEx(bitwise_tubo, bitwise_tubo, MORPH_CLOSE, kernel, Point(-1,-1), 1);

        marcador_ruim = imlimiares(bitwise_tubo, ARR_MINRED, ARR_MAXRED);

    }


    Mat imlimiares(Mat img, int hsvMin[3], int hsvMax[3])
    {
        Mat grayImage;

        inRange(img, Scalar(hsvMin[0], hsvMin[1], hsvMin[2]), Scalar(hsvMax[0], hsvMax[1], hsvMax[2]), grayImage);

        grayImage = imfill(grayImage);

        return grayImage;
    }


    Mat imfill(Mat img)
    {
        vector<vector<Point>> aux_contours;

        morphologyEx(img, img, MORPH_CLOSE, kernel, Point(-1, -1), 3);

        findContours(img, aux_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	    vector<vector<Point>> hull( aux_contours.size() );

        for (size_t i = 0; i<aux_contours.size(); i++)
        {
            convexHull( aux_contours[i], hull[i] );
        }

        for (size_t i = 0; i<aux_contours.size(); i++)
        {
            drawContours(img, hull, i, 255, -1);
        }

        return img;
    }


    bool found()
    { 
        bool success = false;

        this->processImage();

        findContours(this->marcador_ruim, this->contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        for (int i=0; i<this->contours.size(); i++)
        {
            Rect currentrect = boundingRect( this->contours[i] );

            if ( currentrect.area() <  40)
            {
                continue;
            }
            else
            {
				badmark = currentrect;
				success = true;
            }
        }

        this->found_sensor.data = success;
        return success;
    }


    void draw()
    {
        rectangle(mainImage_C3, badmark, cvCOLOR_RED , 2);
    }


    void show(const char* title)
    {
        imshow(title, this->mainImage_C3);
    }
};


FindSensor sensor;

Mat frame;

ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		sensor.setImage(cv_bridge::toCvShare(msg, "bgr8")->image);

		std_msgs::Bool found_sensor;
		
		// Verifica se achou o sensor
		if ( sensor.found() )
		{
			// Desenha o sensor na imagem
			sensor.draw();

			found_sensor.data = sensor.found_sensor.data;

			pub.publish(found_sensor);
		}  

		// Mostra a imagem
		sensor.show("Sensor_main");
        // imshow("tube", sensor.tubo);
			
		int key = waitKey(20);

		if (key == 32)
		{
			imwrite("/tmp/debug_tube.jpeg", sensor.mainImage_C3);

			key = 255;
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "image_listener");

	ros::NodeHandle n;

	pub = n.advertise<std_msgs::Bool>("/hydrone/sensors_detector/found_sensor", 100);

	image_transport::ImageTransport it(n);
	// Imagem de vídeo
	image_transport::Subscriber sub = it.subscribe("/hydrone/camera_camera/image_raw", 1, imageCallback);
	// Imagem de câmera
	// image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	ros::spin();
		
	destroyWindow("view");
}