// https://github.com/Dovyski/cvui could be also useful for GUI...
#include "Velodyne.h"
#include "CvInc.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <deque>

// min and max might cause incompatibilities...
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif // max
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif // min

using namespace std;

int main(int argc, char* argv[]) 
{
	VELODYNE velodyne;
	double elevation = 0;
	double angles[NB_MEASUREMENTS_VELODYNE];
	double distances[NB_MEASUREMENTS_VELODYNE];


#define MAX_NB_MEASUREMENTS_PER_SCAN_VELODYNE 99999999
	BOOL bNewScan = TRUE;
double alpha_mes_velodyne, d_mes_velodyne;
deque<double> alpha_mes_velodyne_vector;
deque<double> d_mes_velodyne_vector;

int nbprev = (int)alpha_mes_velodyne_vector.size();
int nb = 0;

int j = 0;





	memset(&velodyne, 0, sizeof(VELODYNE));

	ConnectVelodyne(&velodyne, "Velodyne0.txt");
	
	cv::Mat image = cv::Mat::zeros(600, 600, CV_8UC3);

	cv::namedWindow("Lidar Data", cv::WINDOW_AUTOSIZE);

	int counter = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto start = t1;
	for (;;)
	{
		GetDataVelodyne(&velodyne, distances, angles, &elevation);

		counter++;
		if (counter > velodyne.maxhist) 
		{ 
			//image = cv::Mat::zeros(600, 600, CV_8UC3);
			counter = 0; 
		}

		for (int i = 0; i < NB_MEASUREMENTS_VELODYNE; i++)
		{




			alpha_mes_velodyne = angles[i];
			d_mes_velodyne = distances[i];

			if (velodyne.maxhist == 0)
			{
				// Try to detect the beginning of a new scan with the angle discontinuity...
				// Try to be a little bit robust w.r.t. non-decreasing outliers...
				if (((int)alpha_mes_velodyne_vector.size() >= 5)&&
					((alpha_mes_velodyne-alpha_mes_velodyne_vector[(int)alpha_mes_velodyne_vector.size()-5]) > M_PI)&&
					((alpha_mes_velodyne-alpha_mes_velodyne_vector[(int)alpha_mes_velodyne_vector.size()-4]) > M_PI)&&
					((alpha_mes_velodyne-alpha_mes_velodyne_vector[(int)alpha_mes_velodyne_vector.size()-3]) > M_PI)&&
					((alpha_mes_velodyne-alpha_mes_velodyne_vector[(int)alpha_mes_velodyne_vector.size()-2]) > M_PI)&&
					((alpha_mes_velodyne-alpha_mes_velodyne_vector[(int)alpha_mes_velodyne_vector.size()-1]) > M_PI))
					bNewScan = TRUE; else bNewScan = FALSE;
				if (bNewScan)
				{
					// Try to automatically remove old data...
					for (j = nbprev-nb-1; j >= 0; j--)
					{
						if ((int)alpha_mes_velodyne_vector.size() > 0)
						{
							alpha_mes_velodyne_vector.pop_front();
							d_mes_velodyne_vector.pop_front();
							//d_all_mes_velodyne_vector.pop_front();
							//t_velodyne_history_vector.pop_front();
							//xhat_velodyne_history_vector.pop_front();
							//yhat_velodyne_history_vector.pop_front();
							//psihat_velodyne_history_vector.pop_front();
							//vrxhat_velodyne_history_vector.pop_front();
						}
					}
					nbprev = nb;
					nb = 0;
				}
			}

			// For compatibility with a Seanet...
			//d_all_mes_velodyne.clear();
			//d_all_mes_velodyne.push_back(d_mes_velodyne);

			alpha_mes_velodyne_vector.push_back(alpha_mes_velodyne);
			d_mes_velodyne_vector.push_back(d_mes_velodyne);
			//d_all_mes_velodyne_vector.push_back(d_all_mes_velodyne);
			//t_velodyne_history_vector.push_back(tv.tv_sec+0.000001*tv.tv_usec);
			//xhat_velodyne_history_vector.push_back(xhat);
			//yhat_velodyne_history_vector.push_back(yhat);
			//psihat_velodyne_history_vector.push_back(psihat);
			//vrxhat_velodyne_history_vector.push_back(vrxhat);

			if (velodyne.maxhist == 0)
			{
				// Try to automatically remove old data...
				nb++;
				if ((nb <= nbprev)&&((int)alpha_mes_velodyne_vector.size() > 0))
				{
					alpha_mes_velodyne_vector.pop_front();
					d_mes_velodyne_vector.pop_front();
					//d_all_mes_velodyne_vector.pop_front();
					//t_velodyne_history_vector.pop_front();
					//xhat_velodyne_history_vector.pop_front();
					//yhat_velodyne_history_vector.pop_front();
					//psihat_velodyne_history_vector.pop_front();
					//vrxhat_velodyne_history_vector.pop_front();
				}
			}
			if (((velodyne.maxhist > 0)&&((int)alpha_mes_velodyne_vector.size() > velodyne.maxhist))||
				((int)alpha_mes_velodyne_vector.size() > MAX_NB_MEASUREMENTS_PER_SCAN_VELODYNE))
			{
				alpha_mes_velodyne_vector.pop_front();
				d_mes_velodyne_vector.pop_front();
				//d_all_mes_velodyne_vector.pop_front();
				//t_velodyne_history_vector.pop_front();
				//xhat_velodyne_history_vector.pop_front();
				//yhat_velodyne_history_vector.pop_front();
				//psihat_velodyne_history_vector.pop_front();
				//vrxhat_velodyne_history_vector.pop_front();
			}










		}


		image = cv::Mat::zeros(600, 600, CV_8UC3);

		for (int i = 0; i < (int)alpha_mes_velodyne_vector.size(); i++)
		{

			// Convert the azimuth and distance to Cartesian coordinates
			//double x = distances[i] * std::cos(angles[i]);
			//double y = distances[i] * std::sin(angles[i]);
			double x = d_mes_velodyne_vector[i] * std::cos(alpha_mes_velodyne_vector[i]);
			double y = d_mes_velodyne_vector[i] * std::sin(alpha_mes_velodyne_vector[i]);

			// Scale and translate the coordinates to fit in the image
			int pixelX = static_cast<int>(x * 100) + image.cols / 2; // 100 is arbitrary scaling factor
			int pixelY = static_cast<int>(y * 100) + image.rows / 2;

			// Draw the point in the image
			//cv::circle(image, cv::Point(pixelX, pixelY), 1, cv::Scalar(0, 255, 0), -1);
			if (pixelX >= 0 && pixelX < image.cols && pixelY >= 0 && pixelY < image.rows)
				image.at<cv::Vec3b>(pixelY, pixelX) = cv::Vec3b(0, 255, 0);
		}

        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        t1 = t2;
        double fps = 1000.0 / duration;
        std::string fps_text = "FPS: " + std::to_string(fps);
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - start).count();
        std::string time_text = "Elapsed time: " + std::to_string(elapsed_time) + " ms";

        cv::putText(image, fps_text, cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
        cv::putText(image, time_text, cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);

		cv::imshow("Lidar Data", image);
        int key = cv::pollKey(); // See https://forum.opencv.org/t/waitkey-without-releasing-key-imshow-not-updated-and-key-presses-recieved-after-release/11488/10 and https://github.com/opencv/opencv/blob/next/modules/highgui/src/window.cpp for potential limitations for non-Win32 API backends...
        if(key == 'q') {
            break;
        }
	}

	cv::destroyWindow("Lidar Data");

	DisconnectVelodyne(&velodyne);

	return EXIT_SUCCESS;
}
