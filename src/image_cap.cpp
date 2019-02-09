#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

template <typename T>
  string NumberToString ( T Number )
  {
     ostringstream ss;
     ss << Number;
     return ss.str();
  }

int main(int argc, char** argv)
{
for (int i=1;i<=2;i++)
{
    
    VideoCapture cap0;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap0.open(2))
        return 0;
    VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(1))
        return 0;
      for(;;)
      {
          Mat frame0;
          cap0 >> frame0;
          if( frame0.empty() ) break; // end of video stream
          imshow("Right:)", frame0);
          Mat gray_image0;
          cvtColor( frame0, gray_image0, CV_BGR2GRAY );
          Mat frame;
          cap >> frame;
          if( frame.empty() ) break; // end of video stream
          imshow("Left:)", frame);
          Mat gray_image;
          cvtColor( frame, gray_image, CV_BGR2GRAY );
          string savingName0 = "Calibration/right" + NumberToString (i) + ".jpg";
          string savingName = "Calibration/left" + NumberToString (i) + ".jpg";
          //cout<< savingName;
          imwrite( savingName, gray_image );
          imwrite( savingName0, gray_image0 );
          
          if (waitKey(60) == 27) break;
    }
}
//break;
return 0;
}
