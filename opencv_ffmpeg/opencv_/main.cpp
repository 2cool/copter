#
	//VideoCapture cap("d:/3.avi");
	//VideoCapture cap; \
		if (!cap.open(0))\
			return 0;


	//VideoCapture cap;
	//cap.open("udp://localhost:554");

	// HARRIS CORNERS TRACKER

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <string>


using namespace cv;
using namespace std;




void MyLine(Mat img, Point start, Point end)
{
	int thickness = 1;
	int lineType = 8;
	line(img,
		start,
		end,
		Scalar(255, 255, 255),
		thickness,
		lineType);
}


int main(int argc, char* argv[])
{


	string line;
	ifstream myfile("D:/torrents/vid/in_.txt");
	
	string windowName = "Harris Corner Detector";

	// Current frame
	Mat frame;


	//"videoout1.txt"
	
	//VideoCapture cap("C:/ffmpeg/bin/videoin.mp4");
	VideoCapture cap("D:/torrents/vid/in__.avi");

	if (!cap.isOpened())
	{
		cerr << "Unable to open the webcam. Exiting!" << endl;
		return -1;
	}
	getline(myfile, line);

	// Iterate until the user presses the Esc key

		// Capture the current frame


		//2,-1,16,16,   8,   8,   8,   8,0x0

	int old_frame = 1, frameN = 2, testF = 1;



		cap >> frame;

		while (true) {
			getline(myfile, line, ',');
			frameN = stoi(line);
			if (old_frame != frameN) {
				// Showing the result
				old_frame = frameN;
				imshow(windowName, frame);
				testF++;
				while (frameN != testF) {
					cap >> frame;
					imshow(windowName, frame);
					testF++;
				}
				

				cap >> frame;
				// Get the keyboard input and check if it's 'Esc'
				// 27 -> ASCII value of 'Esc' key
				char ch = waitKey(1);
				if (ch == 27) {
					break;
				}
			}

			getline(myfile, line, ',');
			getline(myfile, line, ',');
			getline(myfile, line, ',');


			getline(myfile, line, ',');
			int x0 = stoi(line);
			getline(myfile, line, ',');
			int y0 = stoi(line);
			getline(myfile, line, ',');
			int x1 = stoi(line);
			getline(myfile, line, ',');
			int y1 = stoi(line);
			getline(myfile, line);

			Point p1(x0, y0);
		//	if ((x0 - 8) % 16 || (y0 - 8) % 16)
		//		break;
			Point p2(x1, y1);
			MyLine(frame, p1, p2);
			}
			
		
	

	// Release the video capture object
	cap.release();

	// Close all windows
	destroyAllWindows();

	return 1;
}


