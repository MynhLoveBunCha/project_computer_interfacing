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
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

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
const int N_ALLOW_PEOPLE = 2;
VideoCapture cap;
int server_fd, new_socket, valread, serial_port;
Mat send_frame;

void communication(){
	while(1)
	{	
		mtx.lock();
		send(new_socket, (char *)(send_frame.data), 3*CAM_WIDTH*CAM_HEIGHT, 0);
		mtx.unlock();
		this_thread::sleep_for(std::chrono::milliseconds(100));

		printf("--------------------------------------------\n");
		//wait for number of people back from client (wait acknownledge)
		char buffer_info[4] = "";
		valread = read(new_socket, buffer_info, 4);
		int* people_ptr = (int*)(&buffer_info[0]);
		int person_count = people_ptr[0];
		printf("Detect %i people\n", person_count);

		// send open/close signal to uart
		char write_buf[256];
		if(person_count >= N_ALLOW_PEOPLE){
			strcpy(write_buf, "1");
		}
		else{
			strcpy(write_buf, "0");
		}
		int len = strlen(write_buf);
		int n_bytes = write(serial_port, &write_buf, len);
		printf("Sent %i bytes over UART\n", n_bytes);

        // Allocate memory for read buffer
        char read_buf [256];
        memset(&read_buf, '\0', sizeof(read_buf));

        // read
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        // check if any error
        if (num_bytes < 0) {
            printf("Error reading: %s\n", strerror(errno));
            break;
        }

		if(read_buf[0] == '1'){
        	printf("Read %i bytes. Received message: %s ~ OPEN\n", num_bytes, read_buf);
		}
		else{
        	printf("Read %i bytes. Received message: %s ~ CLOSE\n", num_bytes, read_buf);
		}
    


		if(valread <= 0) 
		{
			printf("Connection is closed, Wait a new connection\n");
			break;
		}
	}
}


void camera_opencv()
{
	//--- INITIALIZE VIDEOCAPTURE
    
	int deviceID = 0;             // 0 = open default camera
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

	// Get frame
	while(1){
		mtx.lock();
		cap.read(send_frame);
		mtx.unlock();
		this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}


int init_serial(){
	// open serial port
    int local_serial_port = open("/dev/ttyUSB0", O_RDWR);

    // create termios struct
    struct termios tty;

    // read in existing setting and handle any error
    if(local_serial_port < 0){
        cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return -1;
    }
    tcgetattr(local_serial_port, &tty);

    // Setting
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication 
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte 
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;  // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 5;    // Wait for up to 0.5s (5 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    tcflush(local_serial_port, TCIOFLUSH);
    if (tcsetattr(local_serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }
	return local_serial_port;
}


int main(int argc, char const* argv[])
{
	thread thr_opencv(camera_opencv);
	
	serial_port = init_serial();

	if(serial_port < 0){
		return -1;
	}

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
		
		////send image info to client
		int img_info[2] = {CAM_WIDTH, CAM_HEIGHT};
		
		send(new_socket, img_info, 8, 0);
		printf("image info sent \n");
		
		// wait hello message from client
		valread = read(new_socket, buffer, 1024);
		buffer[valread] = 0;
		printf("%s \n", buffer);

		// TODO: thread
		communication();
		// thread comm_thread(communication);

		// closing the connected socket
		close(new_socket);
	}
	// closing the listening socket
	shutdown(server_fd, SHUT_RDWR);
	
	thr_opencv.join();
	return 0;
}
