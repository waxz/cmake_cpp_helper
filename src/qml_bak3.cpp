#include <QQmlApplicationEngine>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QtQml>
#include <QQuickView>
#include <thread>
#include <chrono>
#include "backend.h"

#include <HMM/Hmm.h>
#include <memory>
#include <thread>
//#include <unsupported/Eigen/MPRealSupport>
#include <boost/multiprecision/gmp.hpp>  // Defines the wrappers around the GMP library's types
//#include <boost/multiprecision/mpfr.hpp>  // Defines the Backend type that wraps MPFR
//#include <boost/math/bindings/mpfr.hpp>
namespace mp = boost::multiprecision;

#include <opencv2/opencv.hpp>

//### New Code ###

#include <QQmlContext>

//mpfr

//################



#include <ros/ros.h>


class GuiThreadModel {
private:
    std::thread m_thread;
    QUrl &m_url;

    bool isDone;

public:
    std::shared_ptr <QQmlApplicationEngine> m_engine_ptr;

    std::shared_ptr <CppClass> m_caller_ptr;

    std::shared_ptr <LiveImageProvider> m_img_ptr;

    std::shared_ptr <ShowImage> m_showimg_ptr;

public:
    GuiThreadModel(QUrl &url) : m_url(url), isDone(false) {};

    void initContext(QUrl &url);

    bool initDone();

public:
    void wait();
};

class MQtThread {
    QUrl &m_url;
public:
    explicit MQtThread(QUrl &url);

    void operator()(GuiThreadModel &guiModel);
};


/* one inteface with register class
 * function caller
 *
 * */
void GuiThreadModel::initContext(QUrl &url) {

    std::cout << "========initContext========" << std::endl;

    // create gui engine
    m_engine_ptr = std::make_shared<QQmlApplicationEngine>();

    m_caller_ptr = std::make_shared<CppClass>();

    m_img_ptr = std::make_shared<LiveImageProvider>();

    m_showimg_ptr = std::make_shared<ShowImage>();

    qmlRegisterType<BackEnd>("io.backend", 1, 0, "BackEnd");
    qmlRegisterType<ImageItem>("myextension", 1, 0, "ImageItem");
    std::cout << "========load io.backend========" << std::endl;

    (*m_engine_ptr).rootContext()->setContextProperty("CppClass", m_caller_ptr.get());
    (*m_engine_ptr).rootContext()->setContextProperty("liveImageProvider", m_img_ptr.get());
    std::cout << "========setContextProperty========" << std::endl;
    (*m_engine_ptr).addImageProvider("live", m_img_ptr.get());

    (*m_engine_ptr).rootContext()->setContextProperty("CodeImage", m_showimg_ptr.get());
    (*m_engine_ptr).addImageProvider(QLatin1String("CodeImg"), m_showimg_ptr.get()->m_pImgProvider);

    (*m_engine_ptr).load(url);
    std::cout << "========load url========" << std::endl;


    // check qml load

    isDone = !m_engine_ptr.get()->rootObjects().empty();

    if (!isDone) {
        throw std::logic_error((m_url.toString().toStdString() + " load fail").c_str());
    }
    std::cout << "========initContext DONE ========" << std::endl;


}

bool GuiThreadModel::initDone() {

    return isDone;
}

void GuiThreadModel::wait() {

#if 1

    MQtThread qt1(m_url);

    m_thread = std::thread(qt1, std::ref(*this));
    std::cout << "QtGuiThread create thread" << std::endl;

    m_thread.detach();
//    m_thread.join();

    // block until all gui created ok
#endif
    while (!initDone()) {

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "QtGuiThread initDone" << std::endl;

}


MQtThread::MQtThread(QUrl &url) : m_url(url) {
}

void MQtThread::operator()(GuiThreadModel &guiModel) {
    int argc;
    char **argv{};
    QGuiApplication app(argc, argv);
    guiModel.initContext(m_url);
    app.exec();
}

/*
 * map 0-255 to different model param
 * 0 [[0.9,0.1],[0.6,0.4]]
 *
 *
 *
 * 255 [[0.01,0.99],[0.01, 0.99]]
 * */




int main(int argc, char *argv[]) {

// ros
    ros::init(argc, argv, "test");

    ros::NodeHandle node;

//cv
    auto img1 = cv::imread("/home/waxz/cpp.png", 0);
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
    img1.convertTo(img, CV_32FC1, 1.0 / 255, 0.0);


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
    cv::Mat cvT(4, 4, CV_32FC1);

//directly use the buffer allocated by OpenCV
    Eigen::Map <Eigen::Matrix4f> eigenT(cvT.ptr<float>());
    Eigen::Map <Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat1(img.ptr<double>(), rows,
                                                                                             cols);
//    throw std::logic_error(std::string("mat to matrix convention type not match, ") + std::to_string(img1.type()));



#if 1

    // mat1.col(0).setZero();
    std::cout << "== mat1 size " << mat1.rows() << ", " << mat1.cols() << std::endl;

    mat1.block(500, 500, 30, 30).setZero();
//    std::cout << "=== mat1 matrix" << mat1 << std::endl;

    std::cout << "=== mat1 roi" << mat1.block(500, 500, 30, 30) << std::endl;

    cv::Mat img2(img.rows, img.cols, img.type(), mat1.data());
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

    std::vector <HMM_t> HMM_models(Hmm_num);

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
    exit(0);
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