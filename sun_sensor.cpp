#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <fcntl.h>      // สำหรับ serial
#include <termios.h>    // สำหรับ serial
#include <unistd.h>     // สำหรับ write()
#include <chrono>  // <--- เพิ่มส่วนนี้ที่ด้านบน

#include "rotation_utils.h"
 
using namespace cv;
using namespace std;
 
const String window_capture_name = "Video Capture";
 
float HFOV = 59.98;
float VFOV = 46.80;
 
// === ฟังก์ชันเปิด Serial ===
int openSerial(const char* portname) {
    int serial_port = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port == -1) {
        cerr << "Failed to open serial port " << portname << endl;
        return -1;
    }
 
    termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_port, &tty) != 0) {
        cerr << "Error from tcgetattr" << endl;
        close(serial_port);
        return -1;
    }
 
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
 
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
 
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        cerr << "Error from tcsetattr" << endl;
        close(serial_port);
        return -1;
    }
 
    return serial_port;
}
 
int main(int argc, char* argv[]) {
    std::string pipeline = "libcamerasrc ! video/x-raw,width=640,height=480,framerate=4/1 ! videoconvert ! appsink";
    VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "ERROR: Cannot open video capture\n";
        return -1;
    }
 
    // === เปิด Serial ===
    const char* serial_port_path = "/dev/ttyGS0";
    int serial_fd = openSerial(serial_port_path);
    if (serial_fd < 0) return -1;
 
    Mat frame, undistort_frame, gray_frame, frame_threshold, edges, cal_vector;
    Mat Sunvector = (cv::Mat1d(1, 3) << 0, 0, 1);
    Mat cameraMatrix = (cv::Mat1d(3, 3) << 554.49531308, 0, 319.38624432, 0, 554.59736974, 235.85541156, 0, 0, 1);
    Mat distCoeffs = (cv::Mat1d(1, 5) << -0.04254391, 0.06807537, -0.00280054, 0.00047624, 0.22512133);
 
    while (true) {
        auto start = chrono::high_resolution_clock::now();  // <--- เริ่มจับเวลา
        cap >> frame;
        if (frame.empty()) break;
        
        cvtColor(frame, frame, COLOR_BGR2GRAY);
        undistort(frame, undistort_frame, cameraMatrix, distCoeffs);
        // GaussianBlur(undistort_frame, undistort_frame, Size(21, 21), 0);
        medianBlur(undistort_frame, undistort_frame, 11);
 
        Size size = undistort_frame.size();
        double width = size.width, height = size.height;
        vector<Vec3f> circles;
 
        // cvtColor(undistort_frame, gray_frame, COLOR_BGR2GRAY);
        inRange(undistort_frame, Scalar(240), Scalar(255), frame_threshold);
        Canny(frame_threshold, edges, 100, 200);
        HoughCircles(edges, circles, HOUGH_GRADIENT, 1, edges.rows / 16, 50, 20, 0, 0);
 
        if (!circles.empty()) {
            Vec3f largest_circle = circles[0];
            for (size_t i = 1; i < circles.size(); i++)
                if (circles[i][2] > largest_circle[2])
                    largest_circle = circles[i];
 
            Point center = Point(largest_circle[0], largest_circle[1]);
            int radius = largest_circle[2];
            circle(frame, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
            circle(frame, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
 
            Point2d center_frame((((width / 2) - center.x) / width), (((height / 2) - center.y) / height));
            double azimuth = center_frame.x * HFOV;
            double altitude = center_frame.y * VFOV;
 
            cal_vector = Sunvector * Roty(azimuth) * Rotx(altitude);

            auto end = chrono::high_resolution_clock::now();  // <--- จับเวลาหลังประมวลผล
            double duration_ms = chrono::duration<double, std::milli>(end - start).count();
            cout << "Processing Time: " << duration_ms << " ms" << endl;
 
            // แสดงค่า
            cout << "Azimuth: " << azimuth << ", Altitude: " << altitude << endl;
            cout << "Sun vector: " << cal_vector << endl;
 
            // === สร้างข้อความและส่งผ่าน Serial ===
            std::ostringstream ss;
            ss.precision(5);
            ss << "Azimuth: " << azimuth << ", Altitude: " << altitude
                << ", Vector: [" << cal_vector.at<double>(0) << ", "
                << cal_vector.at<double>(1) << ", "
                << cal_vector.at<double>(2) << "]\n";
            std::string message = ss.str();
            write(serial_fd, message.c_str(), message.length());
        }else{
            cout << "finding the sun"<< endl;
 
            std::ostringstream ss;
            ss.precision(5);
            ss << "finding the sun" << "\n";
            std::string message = ss.str();
            write(serial_fd, message.c_str(), message.length());
        }

        // imshow("window_detection_name", frame);
 
        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) break;
    }
 
    close(serial_fd);
    return 0;
}