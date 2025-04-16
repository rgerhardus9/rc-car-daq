#include <opencv2/opencv.hpp>

#include <iostream>

int main() {
    // Open the default camera (0). You can change this to your device ID or use a video file.
    cv::VideoCapture cap(0); // or use "/dev/video0"

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // Optional: Set resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
    cap.set(cv::CAP_PROP_FPS, 90); 

    while (true) {
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
        // TODO: Check for extra time/performance
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Show the original and mask
        cv::imshow("Camera", frame);
        cv::imshow("Pink Mask", mask);

        // Press ESC to exit
        if (cv::waitKey(1) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
