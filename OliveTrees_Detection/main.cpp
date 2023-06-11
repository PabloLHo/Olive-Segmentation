#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include "dbscan.hpp"
#include <algorithm>

using namespace cv;
using namespace std;

auto flatten(const std::vector<std::vector<size_t>>& clusters, size_t n)
{
    auto flat_clusters = std::vector<size_t>(n);

    for (size_t i = 0; i < clusters.size(); i++)
    {
        for (auto p : clusters[i])
        {
            flat_clusters[p] = i + 1;
        }
    }

    return flat_clusters;
}

std::vector<int> dbscan2d(const std::span<const float>& data, float eps, int min_pts)
{
    std::vector<int> output;
    auto points = std::vector<point2>(data.size() / 2);

    std::memcpy(points.data(), data.data(), sizeof(float) * data.size());

    auto clusters = dbscan(points, eps, min_pts);
    auto flat = flatten(clusters, points.size());

    for (size_t i = 0; i < points.size(); i++)
    {

        output.push_back(flat[i]);

    }

    return(output);
}

int main(int argc, char **argv)
{
    // READ INPUT IMAGE
    cv::Mat img = cv::imread("input/3.png");


    // BGR TO GRAY SCALE
    cv::Mat greyMat;
    cv::cvtColor(img, greyMat, cv::COLOR_BGR2GRAY);


    // BLURING IMAGE
    cv::blur(greyMat, greyMat, Size(5, 5));


    // CANNY UMBRALIZATION
    Mat CANNY_EDGE_DETECTION;
    int lowThreshold = 30;
    const int kernel_size = 3;
    const int ratio = 3;
    cv::Canny(greyMat, CANNY_EDGE_DETECTION, lowThreshold, lowThreshold * ratio, kernel_size, true);
    cv::imwrite("output/CANNY_EDGE_DETECTION.jpg", CANNY_EDGE_DETECTION);


    // CREATE STRUCTURING ELEMENT
    int morph_size = 2;
    Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));


    // CLOSING - Cierra los contornos
    Mat MORPH_CLOSE_IMAGE;
    morphologyEx(CANNY_EDGE_DETECTION, MORPH_CLOSE_IMAGE, MORPH_CLOSE, element, Point(-1, -1), 1, BORDER_DEFAULT);
    cv::imwrite("output/MORPH_CLOSE_IMAGE.jpg", MORPH_CLOSE_IMAGE);


    // FIND CONTOURS - Busca los contornos
    vector<vector<cv::Point>> allContour, filterContour;
    cv::findContours(MORPH_CLOSE_IMAGE, allContour, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);


    // ESTIMATE THE CENTER POINF OF EACH CONTOUR
    vector<Point> centerContour;
    for (size_t i = 0; i < allContour.size(); i++)
    {
        cv::Moments M = cv::moments(allContour[i]);
        cv::Point center(M.m10 / M.m00, M.m01 / M.m00);
        centerContour.push_back(center);
    }


    // DBSCAN METHOD 
    vector<float> points;
    for (auto p : centerContour) 
    {
        points.push_back(p.x);
        points.push_back(p.y);
    }
    vector<int> clusters = dbscan2d(points, 20, 1);
    //cout << "Clusters detected: " << *max_element(clusters.begin(), clusters.end()) << endl;


    // GROUP POINT PER CLUSTER
    vector<vector<Point>> clustering(*max_element(clusters.begin(), clusters.end()));
    for (int i = 0; i < centerContour.size(); i++) 
        clustering[clusters[i]-1].push_back(centerContour[i]);
 
    
    cv::Mat CLUSTERING_IMAGE;
    img.copyTo(CLUSTERING_IMAGE);
    for (auto cluster : clustering)
    {
        Scalar color = Vec3b(rand() & 255, rand() & 255, rand() & 255);
        for (auto point : cluster)
            cv::circle(CLUSTERING_IMAGE, point, 3, color, -1);
        
    }
    cv::imwrite("output/CLUSTERING_IMAGE.jpg", CLUSTERING_IMAGE);


    // JOIN SIMILAR POINTS
    vector<Point> oliveTrees;
    cv::Mat oliveDetection;
    img.copyTo(oliveDetection);
    for (auto cluster : clustering) {
        oliveTrees.push_back(cluster[0]);
    }

    std::cout << img.cols << "/" << img.rows << "/";

    for (auto olive : oliveTrees) {
        std::cout << olive << "-";
        cv::circle(oliveDetection, olive, 6, Vec3b(rand() & 255, rand() & 255, rand() & 255), 2);
    }

    
    cv::imwrite("output/OLIVE_DETECTION.jpg", oliveDetection);

    std::cout << "Program Finished, We have detected a total of " << oliveTrees.size() << " olive trees" << endl;

    return 0;
}




//// Resize image
//inline cv::Mat resize(const cv::Mat& in, int width)
//{
//    if (in.size().width <= width)
//        return in;
//    float yf = float(width) / float(in.size().width);
//    cv::Mat im2;
//    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
//    return im2;
//}
//
//vector<vector<cv::Point>> contoursFilter(vector<vector<cv::Point>> allContour, int minArea, float _epsilon)
//{
//    vector<vector<cv::Point>> filterContour;
//    // Filter Contours
//    for (int i = 0; i < allContour.size(); i++)
//    {
//        //filterContour.push_back(allContour[i]);
//        vector<cv::Point> approxcontour;
//        double areaContour = cv::contourArea(allContour[i]);
//        // Area Filtrer
//        if (areaContour > minArea)
//        {
//            // Approximate Polygon
//            double epsilon = _epsilon * allContour[i].size();
//            cv::approxPolyDP(allContour[i], approxcontour, epsilon, true);
//
//            // Check border size
//            if (isContourConvex(approxcontour))
//                filterContour.push_back(approxcontour);
//        }
//    }
//    return filterContour;
//}


//// FILTERS CONTOUR
//int minArea = 1;
//vector<vector<cv::Point>> hypothesis;
//filterContour = contoursFilter(allContour, minArea, 0.2);


//// DRAW CONTOURS
//for (int i = 0; i < filterContour.size(); i++)
//{
//    for (int j = 0; j < filterContour[i].size(); j++)
//    {
//        if (j < filterContour[i].size() - 1)
//            cv::line(img, filterContour[i][j], filterContour[i][j + 1], cv::Scalar(0, 0, 255), 4, cv::LINE_8, 0);
//        else
//            cv::line(img, filterContour[i][j], filterContour[i][0], cv::Scalar(0, 0, 255), 4, cv::LINE_8, 0);
//        //cv::putText(img, std::to_string(i), allContour[i][j], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2, cv::LINE_8, 0)
//    }

//}
//cout << "Contornos detectados " << filterContour.size() << endl;
//cv::imwrite("output/conotursImage.jpg", img);



//// Create a structuring element (SE)
//int morph_size = 2;
//Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
//Mat erod, dill;

//// For Dilation
//dilate(thresholdImage, dill, element,
//    Point(-1, -1), 1);

//// For Erosion
//erode(dill, erod, element,
//    Point(-1, -1), 1);


//// Display the image
//imshow("source", thresholdImage);
//imshow("erosion", erod);
//imshow("dilate", dill);

//int thresh = 100;
//RNG rng(12345);

//Mat canny_output;
//Canny(img, canny_output, thresh, thresh * 2);
//imshow("Contours", canny_output);
//cv::waitKey();

//vector<vector<Point> > contours;
//vector<Vec4i> hierarchy;
//findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
//Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);

//vector<Rect> boundRect(contours.size());
//vector<vector<Point> > contours_poly(contours.size());
//vector<Point2f>centers(contours.size());
//vector<float>radius(contours.size());

//for (size_t i = 0; i < contours.size(); i++)
//{
//    approxPolyDP(contours[i], contours_poly[i], 3, true);
//    boundRect[i] = boundingRect(contours_poly[i]);
//    //minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
//}
//drawing = Mat::zeros(canny_output.size(), CV_8UC3);

//for (size_t i = 0; i < contours.size(); i++)
//{
//    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
//    drawContours(drawing, contours_poly, (int)i, color);
//    rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
//    circle(drawing, centers[i], (int)radius[i], color, 2);
//}
//imshow("Contours", resize(drawing, 720));

//cv::waitKey();


//for (size_t i = 0; i < contours.size(); i++)
//{
//    cv::Moments M = cv::moments(contours[i]);
//    cv::Point center(M.m10 / M.m00, M.m01 / M.m00);

//    cv::circle(img, center, 1, cv::Scalar(255, 0, 0), 1);


//    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
//    drawContours(drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
//}
//imshow("Contours", drawing);
//cv::waitKey();