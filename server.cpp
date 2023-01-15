// Server side C/C++ program to demonstrate Socket
// programming
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#define PORT 8080
#define CAM_WIDTH 640
#define CAM_HEIGHT 480

using namespace cv;
using namespace std;

Mat buffer_frame;
mutex mtx;

VideoCapture cap;

void camera_opencv()
{
	//--- INITIALIZE VIDEOCAPTURE
    
	int deviceID = 2;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    cap.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);//Setting the width of the video
    cap.set(CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);//Setting the height of the video//
   
    // check if we succeeded
    if (!cap.isOpened()) {
		
        cerr << "ERROR! Unable to open camera\n";
        return;
    }
	
}

int main(int argc, char const* argv[])
{
	//thread thr_opencv(camera_opencv);
	
	camera_opencv();
	
	int server_fd, new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1024];
		
	string hello = "Hello from server";
	char* hello_ptr = (char *)(&hello[0]);

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET,
				SO_REUSEADDR | SO_REUSEPORT, &opt,
				sizeof(opt))) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr*)&address,
			sizeof(address))
		< 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	if (listen(server_fd, 3) < 0) {
		perror("listen");
		exit(EXIT_FAILURE);
	}
	cout << "Wait a connection ..." << endl;
	
while(1)
{
	if ((new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) 
	{
		perror("accept");
		exit(EXIT_FAILURE);
	}
	cout << "Connection is established" << endl;
	
	////send hello message to client
	// string img_info = to_string(CAM_WIDTH) + " " + to_string(CAM_HEIGHT);
	int img_info[2] = {CAM_WIDTH, CAM_HEIGHT};
	
	send(new_socket, img_info, 8, 0);
	printf("image info sent \n");
	
	// wait hello message from client
	valread = read(new_socket, buffer, 1024);
	buffer[valread] = 0;
	printf("%s \n", buffer);
		
				
	while(1)
	{
			
		Mat send_frame;
		
		cap.read(send_frame);
		cout << "Image rows, cols: " << send_frame.rows << " , " << send_frame.cols << endl;
		
		send(new_socket, (char *)(send_frame.data), 3*CAM_WIDTH*CAM_HEIGHT, 0);
		
		this_thread::sleep_for(std::chrono::milliseconds(100));	
		
		//wait something back from client (wait acknownledge)
		valread = read(new_socket, buffer, 1024);
		if(valread <= 0) 
		{
			cout << "Connection is closed, Wait a new connection" << endl;
			break;
		}
	}
	
	// closing the connected socket
	close(new_socket);
}
	// closing the listening socket
	shutdown(server_fd, SHUT_RDWR);
	
	//thr_opencv.join();
	return 0;
}
