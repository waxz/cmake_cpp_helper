#include <cpp_utils/qt_util.h>
#include <cpp_utils/ros_util.h>
#include <cpp_utils/eigen_util.h>

#include "backend.h"

#include <HMM/Hmm.h>

#include <boost/multiprecision/gmp.hpp>  // Defines the wrappers around the GMP library's types
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

namespace mp = boost::multiprecision;

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
// ros
    ros::init(argc, argv, "test");


    // create laser subscriber
    ros_util::Node node;
    auto scan_ptr = node.createSubscriber<sensor_msgs::LaserScan>("/scan", 1);
    node.getOneMsg("/scan");
#if 1
    sensor_msgs::LaserScan scan;
    scan.angle_min = -1.900;
    scan.angle_max = 1.900;
    int scan_num = 180 * 4;
    scan.angle_increment = (scan.angle_max - scan.angle_min) / (scan_num);
    scan.range_max = 30;
    scan.range_min = 0.0;

    boost::random::mt19937 rng;         // produces randomness out of thin air
    // see pseudo-random number generators
    boost::random::uniform_real_distribution<> six(8.0, 10.0);
    // distribution that maps to 1..6
    // see random number distributions

    for (int i = 0; i < scan_num; i++) {
//        int x = six(rng);

        scan.ranges.push_back(six(rng));
    }

    *scan_ptr = scan;

#endif
    eigen_util::createMatrix<float, Eigen::ColMajor>(&(scan.ranges[0]), 1, scan.ranges.size());
    // create tf listenner
    ros_util::TfListenner tfListenner;
    std::string laser_frame_id = (*scan_ptr).header.frame_id;
    tf::Transform map_base_tf;
    tf::Transform base_laser_tf;

#if 1
//    map_base_tf.setIdentity();

    map_base_tf.setOrigin(tf::Vector3(1.3, 2.4, 0.0));
    map_base_tf.setRotation(tf::createQuaternionFromYaw(3.14 / 4));

    base_laser_tf.setIdentity();
//    base_laser_tf.getOrigin().x() = 1.0;
//    base_laser_tf.getOrigin().y() = 1.5;
#endif

    // ranges to xs ys
    // contour offset
    float contour_offset = 0.05;
    std::valarray<float> xs, ys;
    std::valarray<float> cache_cos, cache_sin;
    std::valarray<float> angles(0.0, (*scan_ptr).ranges.size());
    std::valarray<float> ranges(&(scan.ranges[0]), (*scan_ptr).ranges.size());
    for (int i = 0; i < (*scan_ptr).ranges.size(); i++) {
        angles[i] = scan.angle_min + i * scan.angle_increment;
    }
    cache_cos = cos(angles);
    cache_sin = sin(angles);
    xs = ranges * cache_cos;
    ys = ranges * cache_sin;

    std::valarray<float> xs_offset, ys_offset;
    xs_offset = (ranges - contour_offset) * cache_cos;
    ys_offset = (ranges - contour_offset) * cache_sin;

    // points to contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> contour;

    // push contour to contour vector
    // get local grid size
    float local_x_min = xs.min();
    float local_y_min = ys.min();
    float local_x_max = xs.max();
    float local_y_max = ys.max();

    local_x_min = std::fmin(local_x_min, 0.0);
    float grid_resolution = 0.05;


    int local_height = static_cast<int>(round((local_x_max - local_x_min) / grid_resolution));
    int local_width = static_cast<int>(round((local_y_max - local_y_min) / grid_resolution));


    printf("1 %f, 2 %f, 3 %f, 4 %f", local_x_min, local_y_min, local_x_max, local_y_max);
    std::cout << "create grid " << "width: " << local_width << "height: " << local_height << std::endl;

    cv::Mat local_grid(local_height, local_width, CV_8UC1, cv::Scalar(0));
    std::cout << "mat " << local_grid.rows << ", " << local_grid.cols << std::endl;
    cv::Point p0(static_cast<int>(round((-local_y_min) / grid_resolution)),
                 static_cast<int>(round((-local_x_min) / grid_resolution)));

    contour.push_back(p0);//your cont
    std::vector<double> occuPoints;

    for (int i = 0; i < (*scan_ptr).ranges.size(); i++) {
        cv::Point p(static_cast<int>(round((ys_offset[i] - local_y_min) / grid_resolution)),
                    static_cast<int>(round((xs_offset[i] - local_x_min) / grid_resolution)));


        contour.push_back(p);//your cont

        local_grid.at<uchar>(p) = 255;
        occuPoints.push_back(static_cast<double>(xs[i]));
        occuPoints.push_back(static_cast<double>(ys[i]));

    }

    contours.push_back(contour);

    // fill free lable


    std::vector<double> freePoints;
    {
        cv::Scalar free_lable(100);

        cv::drawContours(local_grid, contours, 0, free_lable, CV_FILLED); // 0: index of contours,
        cv::Rect rect = boundingRect(contours[0]);
        int left = rect.x;
        int top = rect.y;
        int width = rect.width;
        int height = rect.height;
        int x_end = left + width;
        int y_end = top + height;


        for (size_t x = left; x < x_end; x++) {
            for (size_t y = top; y < y_end; y++) {
                cv::Point p(x, y);
                if (100 == local_grid.at<uchar>(p)) {
                    local_grid.at<uchar>(y, x) = 255;
                    freePoints.push_back(static_cast<double>(x * grid_resolution - local_x_min));
                    freePoints.push_back(static_cast<double>(y * grid_resolution - local_y_min));


                }
            }
        }

    }

    auto occuPointsMat = eigen_util::createMatrix<double, Eigen::ColMajor>(&(occuPoints[0]), 2, occuPoints.size() / 2);

    auto freePointsMat = eigen_util::createMatrix<double, Eigen::ColMajor>(&(freePoints[0]), 2, freePoints.size() / 2);

//    std::cout << "occuPointsMat  \n" << occuPointsMat<< std::endl;
//    std::cout << "freePointsMat  \n" << freePointsMat<< std::endl;



    // move to global grid
    // rotate matrix
    cv::Point rotate_center(88, 45);
    float yaw = tf::getYaw(map_base_tf.getRotation());
    auto rotateMat = cv::getRotationMatrix2D(p0, yaw * 180 / M_PI, 1.0);
    rotateMat.at<double>(0, 2) += 20;
    rotateMat.at<double>(1, 2) += 30;
    std::cout << "==== rotate matrix: \n" << rotateMat << std::endl;
    std::cout << "p0: " << p0.x << ", " << p0.y << std::endl;
    // test grid
    cv::Mat global_grid(local_grid.rows + 250, local_grid.cols, CV_8UC1, cv::Scalar(0));
//    cv::warpAffine(local_grid, global_grid, rotateMat, global_grid.size(), cv::INTER_NEAREST);

    eigen_util::TransformationMatrix2d transMat(6, 0.0, 3.14159 / 4);
    occuPointsMat = transMat * occuPointsMat;
    for (int i = 0; i < occuPointsMat.cols(); i++) {
        cv::Point p(static_cast<int>(round((occuPointsMat(1, i) - local_y_min) / grid_resolution)),
                    static_cast<int>(round((occuPointsMat(0, i) - local_x_min) / grid_resolution)));


        cv::circle(global_grid, p, 3, cv::Scalar(255));

    }

    // read map data
    // covert to matrix
    // make mask of dynamic area

    // create 3 matrix
    // one for p(o) , represent te probobility to be occupied
    // one for dynamic mask
    // one for update mask

    //1) lable point in laser scan detecting area
    // free = 0, occupy = 1, unobserve = 2
    // three point set
    // free set, occupy set, unobserve = dynamic area - free set - occupy set

    while (1) {
        cv::imshow("local_grid", local_grid);
        cv::imshow("global_grid", global_grid);

        cv::waitKey(0);
    }


    // get laserscan data
    // ranges to xs, ys
    //convert to contour

    // create local obsevation data grid
    // assign points inside contour
    // get observe grid

    // create global obsevation data grid

    // get robot pose
    // rotate to local grid to global grid
    // update contour cell


    // collect observation sequence of each cell

    // training



    return 0;

    // qt
    QVariantMap qmap;
    qmap["n1"] = 23.6;
    QVariantList qlist;
    qlist << "33" << 3 << 4.34;
    qmap["n2"] = qlist;
//    ros::NodeHandle node;





    // scan to grid
    // points to countor intrest region


//cv
    auto img1 = cv::imread("/home/waxz/cpp.png", 1);
//    cv::Mat img1(5,5,CV_32F,cv::Scalar(0.3));
//
//    int dex = 0;
//    for (int i = 0; i < 5 ; i++){
//        for (int j = 0; j < 5 ; j++){
//            dex ++;
//            img1.at<float>(i,j) = dex;
//
//        }
//    }

//    std::cout << "==debug " << img1  << std::endl;

    cv::Mat img(img1.size(), CV_32F);
    img1.convertTo(img, CV_32FC3, 1.0 / 255, 0.0);


//    cv::imshow("ww",img);

//    std::cout << img << std::endl;

    int cols = img.cols * img.channels();
    int rows = img.rows;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat(rows, cols);

    std::cout << "=== size: " << rows << ", " << cols << std::endl;
    std::cout << "== img  " << img.rows << ", " << img.cols << std::endl;
    std::cout << "== mat " << mat.rows() << ", " << mat.cols() << std::endl;
    std::cout << "=== type: " << img.type() << std::endl;
//    std::cout << "=== img data : " << img(cv::Range(0,10), cv::Range(0,10)) << std::endl;


    //allocate memory for a 4x4 float matrix
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat1(img.ptr<double>(), rows,
                                                                                            cols);
//    throw std::logic_error(std::string("mat to matrix convention type not match, ") + std::to_string(img1.type()));
    cv::Mat img2(img.rows, img.cols, img.type(), mat1.data());


    cv::Point cont1(0, 0);
    cv::Point cont2(100, 0);
    cv::Point cont3(100, 100);
    cv::Point cont4(0, 100);


    int res = 20;
    for (int i = 0; i < scan_num; i++) {
        auto p = cv::Point(round(ys[i] * res) + 500, round(xs[i] * res) + 20);
        contour.push_back(p);//your cont

        cv::circle(img2, p, 3, cv::Vec<float, 3>(0, 0, 1));
        img2.at<cv::Vec<float, 3>>(p.y, p.x) = cv::Vec<float, 3>(0, 0, 1);
    }

    contours.push_back(contour);

    time_util::Timer t1;
    t1.start();
    cv::Mat output(img.rows, img.cols, CV_8UC1);//your img
    cv::Scalar color(255, 0, 0);
#if 1
    for (int i = 0; i < 10; i++) {
        drawContours(img2, contours, 0, color, CV_FILLED); // 0: index of contours,
        cv::Rect rect = boundingRect(contours[0]);
        int left = rect.x;
        int top = rect.y;
        int width = rect.width;
        int height = rect.height;
        int x_end = left + width;
        int y_end = top + height;
        std::vector<cv::Point> blob;
        blob.reserve(width * height);
        for (size_t x = left; x < x_end; x++) {
            for (size_t y = top; y < y_end; y++) {
                cv::Point p(x, y);
                if (255 == img2.at<cv::Vec<float, 3>>(p)[0]) {
                    img2.at<cv::Vec<float, 3>>(y, x) = cv::Vec<float, 3>(255, 0, 255);

                }
            }
        }
        for (size_t y = top; y < y_end; y++) {
            auto ptr = img2.ptr<cv::Vec<float, 3>>(left, top);
            for (size_t x = left; x < x_end; x++) {
//                cv::Point p(x, y);
//                if (255 == img2.at<cv::Vec<float, 3>>(p)[0])
                if ((*ptr)[0] == 255) {
//                    img2.at<cv::Vec<float, 3>>(y, x) = cv::Vec<float, 3>(255,0,255);
                    *ptr = cv::Vec<float, 3>(255, 0, 255);;

                }
                ptr++;
            }
        }
#if 0
        for (int y = top; y < y_end; y++){
        //for(int y = 0; y < img2.rows; y++){
            const  float* ptr = img2.ptr<float>(y);
            for (int x = left; x < x_end; x++){

//            for(int x = 0; x < img2.cols; x++){

                const float * pixel = ptr;
                if(int(pixel[0]) == 255){
                    //point is inside contour
                    img2.at<cv::Vec<float, 3>>(y, x) = cv::Vec<float, 3>(0,255,255);

                }
                ptr += 3;

            }
        }
#endif
    }
#endif


    t1.stop();
    std::cout << "draw contour time " << t1.elapsedSeconds() << std::endl;
#if 1

    // mat1.col(0).setZero();
    std::cout << "== mat1 size " << mat1.rows() << ", " << mat1.cols() << std::endl;

//    mat1.block(500,500,30,30).setZero();
//    std::cout << "=== mat1 matrix" << mat1 << std::endl;

//    std::cout << "=== mat1 roi" << mat1.block(500,500,30,30) << std::endl;

//    std::cout << "==== mat roi" << img2(cv::Range(500,510), cv::Range(500,510)) << std::endl;
//    std::cout << "==== mat matrix" << img2 << std::endl;

//    img = img2;
//    img2.copyTo(img);
//    img2(cv::Range(500,600), cv::Range(500,600)) = 20;
//    return 0;

    while (true) {
//        std::cout << "img1 \n" << img << std::endl;
//        std::cout << "img2 \n" << img2 << std::endl;


//        break;
        cv::imshow("m1", img);
        cv::imshow("m2", img2);
        cv::waitKey(0);
    }
#endif
//    return 0;

#if 0
    boost::multiprecision::mpz_int mm("1238192389824723487823749827349879872342834792374897923479");

    mp::mpf_float nn = 0.01;
    std::cout << pow(mm, 3) << "\n";
    std::cout << pow(nn, 300) << "\n";

    return 0;
    CppClass cppClass;

    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    engine.rootContext()->setContextProperty("CppClass", &cppClass);

    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    return app.exec();
#endif


    //=======================
    auto url1 = QUrl::fromLocalFile("/home/waxz/dev/notebook/cmake_helper/resources/main.qml");

    GuiThreadModel qt1_model(url1);

    // create thread
#if 0
    MQtThread qt1(url1);

    std::thread m_thread(qt1,std::ref(qt1_model));
    std::cout << "QtGuiThread create thread" << std::endl;

    // block until all gui created ok
    std::cout << "QtGuiThread check" << std::endl;

#endif
    qt1_model.wait();

    QVariantList list;
    list << 88 << QColor(Qt::green) << "2233";
    int i = 0;


    //HMM
    // initialise model A B Q Fi Gama
    Eigen::MatrixXf Q(1, STATE_DIM);
    Q << 0.5, 0.5;
    Eigen::MatrixXf A(STATE_DIM, STATE_DIM);
    A << 0.5, 0.5, 0.5, 0.5;

    Eigen::MatrixXf B(STATE_DIM, OBS_DIM);
    B << 0.2, 0.5, 0.3, 0.4, 0.2, 0.4;

    Hmm::TensorX4 Fi(STATE_DIM, STATE_DIM, STATE_DIM, OBS_DIM);
    Hmm::TensorX3 Gama(STATE_DIM, STATE_DIM, OBS_DIM);

    Fi.setZero();
    Gama.setZero();
    Hmm::HmmParams Hmmparams1(Q, A, B, Fi, Gama);
    Hmm::HmmParams Hmmparams2(Q, A, B, Fi, Gama);


    int Hmm_num = 5;
    typedef int HMM_t;

    std::vector<HMM_t> HMM_models(Hmm_num);

    HMM_t hmm_param_1;
    HMM_t hmm_param_2;
    HMM_t hmm_param_3;
    HMM_t hmm_param_4;
    HMM_t hmm_param_5;
    HMM_t hmm_param_6;

    // define params : B, learning_rate, cache_step,


    while (true) {
        // pause trainning
        while (qt1_model.m_caller_ptr.get()->m_sig == 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::cout << "sleep for 1 second" << std::endl;
            // when model 1 click
            // load param to model 1
            if (qt1_model.m_caller_ptr.get()->m_sig == 11) {
                Hmmparams1.B() = qt1_model.m_caller_ptr.get()->m_matrix;
                std::cout << "load data to param 1" << std::endl;
                qt1_model.m_caller_ptr.get()->m_sig = 0;
                continue;
            }


            if (qt1_model.m_caller_ptr.get()->m_sig == 21) {
                Hmmparams2.B() = qt1_model.m_caller_ptr.get()->m_matrix;
                std::cout << "load data to param 2" << std::endl;
                qt1_model.m_caller_ptr.get()->m_sig = 0;

                continue;
            }

        }
        if (qt1_model.m_caller_ptr.get()->m_sig == 2) {
            Hmmparams1.Fi().setZero();
            Hmmparams1.Gama().setZero();
            Hmmparams1.Q() = Q;
            Hmmparams1.A() = A;


            Hmmparams2.Fi().setZero();

            Hmmparams2.Gama().setZero();
            Hmmparams2.Q() = Q;
            Hmmparams2.A() = A;

            std::cout << "reset data " << std::endl;
            qt1_model.m_caller_ptr.get()->m_sig = 0;
            continue;
        }

        if (qt1_model.m_caller_ptr.get()->m_sig == 3) {
            std::cout << "param1: A=\n" << Hmmparams1.A() << std::endl;
            std::cout << "param2: A=\n" << Hmmparams2.A() << std::endl;
            qt1_model.m_caller_ptr.get()->m_sig = 0;
            continue;
        }


        std::cout << " ==== run" << std::endl;
        int bz = 50;
        bool first = true;
        for (int i = 0; i < bz; i++) {
//        cout << "=== " << i << endl;
            int cs = (first) ? 100 : 0;
            first = false;
            time_util::Timer timer1;
            timer1.start();
            Hmm::learn(Hmmparams1,
                       qt1_model.m_caller_ptr.get()->m_sample_data,
                       qt1_model.m_caller_ptr.get()->m_dict["lr"],
                       cs);
            Hmm::learn(Hmmparams2,
                       qt1_model.m_caller_ptr.get()->m_sample_data,
                       qt1_model.m_caller_ptr.get()->m_dict["lr"],
                       cs);
            timer1.stop();
#if 0
            std::cout << "one loop run time: " << timer1.elapsedMicroseconds() << std::endl;

            std::cout << "param 1:\n" << Hmmparams1.A() << std::endl;
            std::cout << "param 2:\n" << Hmmparams2.A() << std::endl;
#endif
        }
        std::cout << "trining done " << std::endl;

        std::cout << "check accuracy" << std::endl;


        auto pred1 = Hmm::predict(Hmmparams1, qt1_model.m_caller_ptr.get()->m_sample_data);
        auto pred2 = Hmm::predict(Hmmparams2, qt1_model.m_caller_ptr.get()->m_sample_data);

        std::valarray<int> pred1_val(&(pred1[0]), pred1.size());
        std::valarray<int> pred2_val(&(pred2[0]), pred1.size());
        std::valarray<int> gt_val(&(qt1_model.m_caller_ptr.get()->m_sample_data[0]), pred1.size());

        float err1 = float((abs(pred1_val - gt_val)).sum()) / pred1.size();
        float err2 = float((abs(pred2_val - gt_val)).sum()) / pred1.size();

        std::cout << "error1 " << err1 << std::endl;
        std::cout << "error2 " << err2 << std::endl;
#if 1
        for (int z = 0; z < pred1.size(); z++) {
            std::cout << "[ " << qt1_model.m_caller_ptr.get()->m_sample_data[z]
                      << ", "
                      << pred1_val[z]
                      << ", "
                      << pred2_val[z]
                      << " ]\n";

        }
#endif
        std::cout << "======" << std::endl;
        std::cout << "param1: A=\n" << Hmmparams1.A() << std::endl;
        std::cout << "param2: A=\n" << Hmmparams2.A() << std::endl;


        if (qt1_model.m_caller_ptr.get()->m_sig == -1) {
            break;
        }


        std::this_thread::sleep_for(std::chrono::seconds(1));
        break;

    }

    cv::cvtColor(img1, img1, CV_BGR2RGB);
    std::cout << "step " << img1.step << std::endl;
//    exit(0);
    // rgb image
    QImage image(img1.data, img1.cols, img1.rows, img1.step, QImage::Format_RGB888);

//    QImage imag2(mat1.data(),mat1.rows(), mat1.cols(),QImage::Format_Grayscale8);

    while (i < 300) {
        i++;

//        qt1_model.m_engine_ptr.get()->


        QMetaObject::invokeMethod(qt1_model.m_engine_ptr.get()->rootObjects()[0], "readValues", Qt::DirectConnection,
                                  Q_ARG(QVariant, QVariant::fromValue(list))
        );
//        QMetaObject::invokeMethod(qt1_model.m_engine_ptr.get()->rootObjects()[0], "setImage", Qt::DirectConnection,
//                                  Q_ARG(QVariant, QVariant::fromValue(image))
//        );
        qt1_model.m_showimg_ptr.get()->setImage(image);
//        qt1_model.m_caller_ptr.get()->trig();
        std::cout << "data from qml: " << qt1_model.m_caller_ptr.get()->m_matrix << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));




        // create 5 hmm model
        // load param data from text area

        // each hmm has a startpause buttton, reset
        // sig     [Model_Id][Cmd]
        //          11 : model 1, start
        //          10 : model 1 pause
        //          12 : model 1 reset Fi param
        //          13 : model 1 reset Q
        //          14 : model 1 reset gama
        //




    }


    std::cout << "exit" << std::endl;
    return 0;

}