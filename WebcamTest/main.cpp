//

/* Sample code that opens up a camera feed with OpenCV
 *
 * Author: Amy Phung
 * Last edited: 10/5/18
 *
 * REQUIRES OPENCV
 * One-line installation: sudo apt-get install libopencv-dev
 */

#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera, use 1 to open camera on benchtop
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    namedWindow("feed",1); // create a window to display feed
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        imshow("feed", frame); // display camera image on window
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}