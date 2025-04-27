// This script is to test the camera function with C++ and compare the speed to the Python file
// Compile with: g++ cam.cpp -o cam `pkg-config --cflags --libs opencv4`
// May need to install openCV for C++

#include <opencv2/opencv.hpp>

#include <iostream>
#include <chrono>   // For timing

int main() {
    // Open the default camera (0). You can change this to your device ID or use a video file.
    cv::VideoCapture cap(0, cv::CAP_V4L2); // or use "/dev/video0"

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // Set resolution and MJPG format
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 90); 

    auto width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    auto height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "Width: " << width << "\t" "Height: " << height << "\n";

    while (true) {
        
        // Start time
        auto loop_start = std::chrono::high_resolution_clock::now();

        cv::Mat frame, hsv, mask;
        cap >> frame;   // frame reads in cap or cap grabs frame?
        
        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed." << std::endl;
            break;
        }
        
        // Convert to HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // Define HSV range for pink (adjust based on your lighting + tape)
        cv::Scalar lower_pink(140, 30, 170); // adjust as needed
        cv::Scalar upper_pink(180, 255, 255);
        
        // Threshold the HSV image to get only pink colors
        cv::inRange(hsv, lower_pink, upper_pink, mask);
        
        // Optional: Use morphological operations to clean up the mask
        
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        
       
       auto loop_end = std::chrono::high_resolution_clock::now();
       std::chrono::duration<double, std::milli> loop_duration = loop_end - loop_start;
       std::cout << "Loop time: " << loop_duration.count() << " ms" << std::endl;

        // Press ESC to exit
        if (cv::waitKey(1) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
