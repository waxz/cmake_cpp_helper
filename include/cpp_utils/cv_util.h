//
// Created by waxz on 18-11-21.
//

#ifndef EVENTLOOP_CV_UTIL_H
#define EVENTLOOP_CV_UTIL_H

#include <opencv2/opencv.hpp>

#include <cmath>

namespace cv_util {

    template<typename T>
    void createMat(T *p, cv::Mat &src) {
        cv::Mat dst(src.rows, src.cols, src.type(), p);
        src = dst;
    }

    cv::Mat getRotateMatrix2d(cv::Point center, double yaw, cv::Point trans = cv::Point()) {
        auto rotateMat = cv::getRotationMatrix2D(center, yaw * 180 / M_PI, 1.0);

        rotateMat.at<double>(0, 2) += trans.x;
        rotateMat.at<double>(1, 2) += trans.y;
        return rotateMat;
    }


    void rotate(const cv::Mat &src, cv::Mat &dst, double angle, cv::Point trans = cv::Point()) {


        double cos_angle = fabs(cos(angle));
        double sin_angle = fabs(sin(angle));

        int dx = int(src.cols * cos_angle + src.rows * sin_angle);
        int dy = int(src.cols * sin_angle + src.rows * cos_angle);

//        cv::Mat mask(src.cols, src.rows,CV_8U, cv::Scalar(1));


//        cv::Mat tempMat = cv::Mat(dx,dy,CV_32F,cv::Scalar(255,0,0));
//        auto rotateMat = cv::getRotationMatrix2D(cv::Point(src.cols/2, src.rows/2), angle*180/M_PI, 1.0);
//
//        rotateMat.at<double>(0,2) += (src.cols * (cos_angle - 1 )+ src.rows * sin_angle)/2;
//        rotateMat.at<double>(1,2) += (src.cols * sin_angle + src.rows * (cos_angle - 1 ))/2;
        cv::Point t;
        if (trans.x == 0. && trans.y == 0) {
            t = cv::Point((src.cols * (cos_angle - 1) + src.rows * sin_angle) / 2,
                          (src.cols * sin_angle + src.rows * (cos_angle - 1)) / 2);

        } else {
            dx += trans.x - dx / 2;
            dy += trans.y - dy / 2;
//            t = cv::Point((src.cols * (cos_angle - 1 )+ src.rows * sin_angle)/2 + trans.x, (src.cols * sin_angle + src.rows * (cos_angle - 1 ))/2 + trans.y) ;

            t = cv::Point(trans.x - src.cols / 2, trans.y - src.rows / 2);


        }
        auto tempMat = cv::Mat(dy, dx, src.type());

        auto rotateMat = getRotateMatrix2d(cv::Point(src.cols / 2, src.rows / 2), angle, t);

//        std::cout << "rotate util "<< rotateMat << std::endl;
        cv::warpAffine(src, tempMat, rotateMat, tempMat.size(), cv::INTER_NEAREST);
        dst = tempMat;

    }

    void rotateCrop(const cv::Mat &src, cv::Mat &dst, double angle, cv::Point center, cv::Point trans = cv::Point()) {


        double cos_angle = fabs(cos(angle));
        double sin_angle = fabs(sin(angle));

        int dx = int(src.cols * cos_angle + src.rows * sin_angle);
        int dy = int(src.cols * sin_angle + src.rows * cos_angle);

//        cv::Mat mask(src.cols, src.rows,CV_8U, cv::Scalar(1));


//        cv::Mat tempMat = cv::Mat(dx,dy,CV_32F,cv::Scalar(255,0,0));
//        auto rotateMat = cv::getRotationMatrix2D(cv::Point(src.cols/2, src.rows/2), angle*180/M_PI, 1.0);
//
//        rotateMat.at<double>(0,2) += (src.cols * (cos_angle - 1 )+ src.rows * sin_angle)/2;
//        rotateMat.at<double>(1,2) += (src.cols * sin_angle + src.rows * (cos_angle - 1 ))/2;
        cv::Point t;
#if 0

        if (trans.x == 0. && trans.y == 0) {
            t = cv::Point((src.cols * (cos_angle - 1) + src.rows * sin_angle) / 2 ,
                          (src.cols * sin_angle + src.rows * (cos_angle - 1)) / 2 );

        } else {
            dx += trans.x - dx / 2;
            dy += trans.y - dy / 2;
//            t = cv::Point((src.cols * (cos_angle - 1 )+ src.rows * sin_angle)/2 + trans.x, (src.cols * sin_angle + src.rows * (cos_angle - 1 ))/2 + trans.y) ;

            t = cv::Point(trans.x - src.cols / 2, trans.y - src.rows / 2);


        }
#endif
        t = cv::Point((src.cols * (cos_angle - 1) + src.rows * sin_angle) / 2,
                      (src.cols * sin_angle + src.rows * (cos_angle - 1)) / 2);

        auto tempMat = cv::Mat(dy, dx, src.type());

        auto rotateMat = getRotateMatrix2d(center, angle);

//        std::cout << "rotate util "<< rotateMat << std::endl;
        cv::warpAffine(src, tempMat, rotateMat, tempMat.size(), cv::INTER_NEAREST);
        dst = tempMat(cv::Range(center.y - trans.y, center.y + trans.y - 1),
                      cv::Range(center.x - trans.x, center.x + trans.x - 1));

    }

    void imshow(std::string name, cv::Mat img) {
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        cv::imshow(name, img);

    }

    cv::Mat imread(std::string filename, int flag = 0) {
        cv::Mat img = cv::imread(filename, flag);
        if (!img.data)
            throw std::logic_error(filename + " Not Found");
        return img;

    }

    cv::Mat slice(const cv::Mat &mat, int row_start, int row_end, int col_start, int col_end) {
        cv::Mat dst = mat(cv::Range(row_start, row_end), cv::Range(col_start, col_end));
        return dst;

    }

    void filter2d(const cv::Mat &src, const cv::Mat &kernel, cv::Mat &dst, double delta = 0.0,
                  int boardType = cv::BORDER_CONSTANT) {

        //BORDER_ISOLATED or BORDER_CONSTANT
        cv::filter2D(src, dst, src.depth(), kernel, cv::Point(-1, -1), delta, boardType);

    }


}

// convert 2d vector to Mat


#endif //EVENTLOOP_CV_UTIL_H
