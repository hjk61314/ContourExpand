#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>

using namespace std;
using namespace cv;


void expand_polygon(vector<Point> &orignal, vector<Point> &out);

int main()
{
    Mat im = imread("../test.png", 0);

    Mat cont = im.clone();
    Mat original = Mat::zeros(im.rows, im.cols, CV_8UC3);
    Mat smoothed = Mat::zeros(im.rows, im.cols, CV_8UC3);
    imshow("raw",im);


    // contour smoothing parameters for gaussian filter
    int filterRadius = 5;
    int filterSize = 2 * filterRadius + 1;
    double sigma = 10;

    vector<vector<Point> > contours,contours_new;
    vector<Vec4i> hierarchy;
    // find external contours and store all contour points
    findContours(cont, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));
    cout<<"Contour.size: "<<contours.size()<<endl;
    cout<<"hierarchy size: "<<hierarchy.size()<<endl;
    for(size_t j = 0; j < contours.size(); j++)
    {
        //cout<<"contours info:"<<contours[j].size()<<endl<<contours[j]<<endl;
        //cout<<"hierarchy info:"<<endl<<hierarchy[j]<<endl;

        // draw the initial contour shape
        drawContours(original, contours, j, Scalar(0, 255, 0), 1);
        // extract x and y coordinates of points. we'll consider these as 1-D signals
        // add circular padding to 1-D signals
        size_t len = contours[j].size() + 2 * filterRadius;
        size_t idx = (contours[j].size() - filterRadius);
        vector<float> x, y;
        for (size_t i = 0; i < len; i++)
        {
            x.push_back(contours[j][(idx + i) % contours[j].size()].x);
            y.push_back(contours[j][(idx + i) % contours[j].size()].y);
        }
//        Point2d vertex;
//        vertex.x = contours[j].

//        cout<<"Point:"<<x.data()<<","<<y.data()<<endl;

        // filter 1-D signals
        vector<float> xFilt, yFilt;
        GaussianBlur(x, xFilt, Size(filterSize, filterSize), sigma, sigma);
        GaussianBlur(y, yFilt, Size(filterSize, filterSize), sigma, sigma);
        // build smoothed contour
        vector<vector<Point> > smoothContours;
        vector<Point> smooth;
        for (size_t i = filterRadius; i < contours[j].size() + filterRadius; i++)
        {
            smooth.push_back(Point(xFilt[i], yFilt[i]));
        }
        smoothContours.push_back(smooth);

        drawContours(smoothed, smoothContours, 0, Scalar(255, 0, 0), 1);

        cout << "debug contour " << j << " : " << contours[j].size() << ", " << smooth.size() << endl;



        vector<Point> temp;
        approxPolyDP(contours[j], temp, 30, true);
        cout<<"contour_new:"<<j<<endl<<temp<<endl;
        contours_new.push_back(temp);//smoothContours.push_back(temp);
        drawContours(smoothed, contours_new, 0, Scalar(0, 0, 255), 1);        
    }

    // draw key vertex after approxPolyDP
    Point vertex;
    for(int i=0; i<contours_new[1].size();i++){
        vertex.x = contours_new[1][i].x;
        vertex.y = contours_new[1][i].y;
        circle(smoothed,vertex,2,Scalar(0,0,255),1);
        ostringstream indexText;// or std::to_string()
//        indexText << i;
//        putText(smoothed,indexText.str(),vertex,cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 1);
    }

    vector<Point> points_expand;
    expand_polygon(contours_new[1],points_expand);
    cout<<endl<<"points_expand:"<<points_expand<<endl;

    //Point vertex;
    for(int i=0; i<points_expand.size();i++){
        vertex.x = points_expand[i].x;
        vertex.y = points_expand[i].y;
        circle(smoothed,vertex,2,Scalar(0,255,0),1);
        ostringstream indexText;// or std::to_string()
        indexText << i;
        putText(smoothed,indexText.str(),vertex,cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0,255 ), 1);
        int next = (i==(points_expand.size()-1) ? 0: (i+1));
        line(smoothed,points_expand[i],points_expand[next],Scalar(0,255,0),1);
    }

    //imshow("original",original);
    imshow("smoothed",smoothed);
    waitKey(0);
}


// Ref 多边形或轮廓等距离外扩或收缩
// https://blog.csdn.net/hjk61314/article/details/82112610
void expand_polygon(vector<Point> &pList, vector<Point> &out){// already ordered by anticlockwise

    // 1. vertex set
    // pList

    // 2. edge set and normalize it
    vector<Point2f> dpList, ndpList;
    int count = pList.size();
    for(int i = 0; i < count; i++){
        int next = (i==(count-1) ? 0: (i+1));
        dpList.push_back(pList.at(next)-pList.at(i));
        float unitLen = 1.0f/sqrt(dpList.at(i).dot(dpList.at(i)));
        ndpList.push_back(dpList.at(i) * unitLen);
        cout<<"i="<<i<<",pList:"<<pList.at(next)<<","<<pList.at(i)<<",dpList:"<<dpList.at(i)<<",ndpList:"<<ndpList.at(i)<<endl;
    }

    // 3. compute Line
    float SAFELINE = 10.0f;//负数为内缩， 正数为外扩。 需要注意算法本身并没有检测内缩多少后折线会自相交，那不是本代码的示范意图
    for(int i = 0; i < count; i++){
        int startIndex = (i==0 ? (count-1):(i-1));
        int endIndex = i;
        float sinTheta = ndpList.at(startIndex).cross(ndpList.at(endIndex));
        Point2f orientVector = ndpList.at(endIndex) - ndpList.at(startIndex);//i.e. PV2-V1P=PV2+PV1
        Point2f temp_out;
        temp_out.x = pList.at(i).x + SAFELINE/sinTheta * orientVector.x;
        temp_out.y = pList.at(i).y + SAFELINE/sinTheta * orientVector.y;
        out.push_back(temp_out);
    }
    //cout<<endl<<"out:"<<out<<endl;

}



