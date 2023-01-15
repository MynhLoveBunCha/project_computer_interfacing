// Client side C/C++ program to demonstrate Socket
// programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <sstream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#define PORT 8080


using namespace cv;
using namespace std;


// set global params
const vector<Scalar> colors = {Scalar(255, 255, 0), Scalar(0, 255, 0), Scalar(0, 255, 255), Scalar(255, 0, 0)};
const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.2;
const float NMS_THRESHOLD = 0.4;
const float CONFIDENCE_THRESHOLD = 0.6;
const int N_ALLOW_PEOPLE = 3;


struct Detection
{
    int class_id;
    float confidence;
    Rect box;
};


vector<string> load_class_list()
{
    vector<string> class_list;
    ifstream ifs("classes.txt");
    string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }
    return class_list;
}


Mat format_yolov5(const Mat &source){
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    Mat result = Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(Rect(0, 0, col, row)));
    return result;
}


void detect(Mat &image, dnn::Net &net, vector<Detection> &output, const vector<string> &className) {
    Mat blob;
    auto input_image = format_yolov5(image);
    dnn::blobFromImage(input_image, blob, 1./255., Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);
    net.setInput(blob);
    std::vector<Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;
    
    float *data = (float *)outputs[0].data;

    const int dimensions = 85;
    const int rows = 25200;
    
    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;

    for (int i = 0; i < rows; ++i) {

        float confidence = data[4];
        if (confidence >= CONFIDENCE_THRESHOLD) {

            float * classes_scores = data + 5;
            Mat scores(1, className.size(), CV_32FC1, classes_scores);
            Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > SCORE_THRESHOLD) {

                confidences.push_back(confidence);

                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(Rect(left, top, width, height));
            }

        }

        data += 85;

    }

    vector<int> nms_result;
    dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }
}


int main(int argc, char const* argv[])
{
		
	
	int sock = 0, valread, client_fd;
	struct sockaddr_in serv_addr;
	string hello = "Hello from client";
	char* hello_ptr = (char *)(&hello[0]);
	char *buffer;
	
	
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("\n Socket creation error \n");
		return -1;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);

	// Convert IPv4 and IPv6 addresses from text to binary form
	if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
		printf(
			"\n Invalid address/ Address not supported \n");
		return -1;
	}

	if ((client_fd
		= connect(sock, (struct sockaddr*)&serv_addr,
				sizeof(serv_addr)))
		< 0) {
		printf("\n Connection Failed \n");
		return -1;
	}


	// get image info from server
	char buffer_info[8] = "";
	valread = read(sock, buffer_info, 8);
	int* info_img = (int*)(&buffer_info[0]);
	int width = info_img[0];
	int height = info_img[1];
	
	// send hello message to server
	send(sock, hello_ptr, hello.length(), 0);
	printf("Hello message sent\n");
	
	buffer = (char *)malloc(3*height*width*sizeof(char)); //contain all the bytes in the image
	Mat received_frame = Mat(height, width, CV_8UC3);


	// load model //
    auto net = dnn::readNet("yolov5s.onnx");

    // Comment the following 3 lines if you do not build opencv with CUDA backend
    // ---------------------------
    cout << "Attempt to use CUDA\n";
    net.setPreferableBackend(dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(dnn::DNN_TARGET_CUDA);
    // ---------------------------


    // Get list of class name
    vector<string> class_list = load_class_list();


	while(1)
	{		
		valread = 0;
		do
		{	
			valread += read(sock, buffer+valread, 3*height*width);
			
		}while(valread < 3*height*width);	

		
		int person_count = 0;
		if(valread == 3*height*width)
		{	
			uchar *ptr = (uchar *)(buffer);
			memcpy(received_frame.data, ptr, 3*height*width);
			// 
			auto start = chrono::high_resolution_clock::now();


			// Inference 
			vector<Detection> output;
			detect(received_frame, net, output, class_list);
			int n_detections = output.size();
			// Draw bounding box
			for (int i = 0; i < n_detections; ++i)
			{

				auto detection = output[i];
				auto classId = detection.class_id;
				string class_name = class_list[classId];

				if(class_name == "person"){
					person_count++;
					auto box = detection.box;
					auto confi = detection.confidence;
					const auto color = colors[classId % colors.size()];
					rectangle(received_frame, box, color, 3);
					rectangle(received_frame, Point(box.x, box.y - 20), Point(box.x + box.width, box.y), color, FILLED);
					putText(received_frame, class_name + " ID: " + to_string(person_count), Point(box.x, box.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
					putText(received_frame, to_string((int)(confi * 100.)) + "%", Point(box.x + 120, box.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
				}

			}
			auto end = chrono::high_resolution_clock::now();

			// FPS calculation
			float fps = 1 * 1000.0 / chrono::duration_cast<chrono::milliseconds>(end - start).count();
			ostringstream fps_label;
			fps_label << fixed << setprecision(2);
			fps_label << "FPS: " << fps;
			string fps_label_str = fps_label.str();

			cv::putText(received_frame, fps_label_str.c_str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
			
			// Display 
			imshow("Image from Server", received_frame);
		}

		// send open/close signal to uart
		// TODO
		// if(person_count == N_ALLOW_PEOPLE){

		// }
		//--------------------------------

		// Exit condition
        if(waitKey(15) >= 0){
            break;
        }
		
		//send something back to server (send acknownledge)
		send(sock, hello_ptr, hello.length(), 0);	
	}

	// closing the connected socket
	close(client_fd);
	return 0;
}
