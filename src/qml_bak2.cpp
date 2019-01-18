#include <QQmlApplicationEngine>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QtQml>
#include <QQuickView>
#include <thread>
#include <chrono>
#include "backend.h"


#include <memory>
#include <thread>

//### New Code ###

#include <QQmlContext>


//################




class GuiThread {

public:

    void operator()(std::shared_ptr <CppClass> &caller_ptr, std::shared_ptr <QQmlApplicationEngine> &engine_ptr) {
        caller_ptr = std::make_shared<CppClass>();
//        CppClass cppClass;
//        *caller_ptr = cppClass;
        int argc;
        char **argv{};
        QGuiApplication app(argc, argv);


        QQmlApplicationEngine engine;
        engine_ptr = std::make_shared<QQmlApplicationEngine>();
        qmlRegisterType<BackEnd>("io.backend", 1, 0, "BackEnd");

        (*engine_ptr).rootContext()->setContextProperty("CppClass", caller_ptr.get());

        (*engine_ptr).load(QUrl(QStringLiteral("qrc:/main.qml")));
        QObject *rootObject = (*engine_ptr).rootObjects()[0];


        // call qml function from c++
        //https://stackoverflow.com/questions/35749873/how-to-pass-a-list-vector-or-array-as-argument-in-invokemethod-function
        QString ret;

        QVariantList list;
        list << 10 << QColor(Qt::green) << "bottles";
        QMetaObject::invokeMethod(rootObject, "readValues", Qt::DirectConnection,
                                  Q_ARG(QVariant, QVariant::fromValue(list))
        );


        std::cout << "get ret: " << ret.toStdString() << std::endl;

        app.exec();

    }
};

class GuiThreadModel {
private:
    std::thread m_thread;
    QUrl &m_url;

    bool isDone;

public:
    std::shared_ptr <QQmlApplicationEngine> m_engine_ptr;

    std::shared_ptr <CppClass> m_caller_ptr;

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
    MQtThread(QUrl &url);

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


    qmlRegisterType<BackEnd>("io.backend", 1, 0, "BackEnd");
    std::cout << "========load io.backend========" << std::endl;

    (*m_engine_ptr).rootContext()->setContextProperty("CppClass", m_caller_ptr.get());
    std::cout << "========setContextProperty========" << std::endl;

    (*m_engine_ptr).load(url);
    std::cout << "========load url========" << std::endl;

    std::cout << "========initContext DONE ========" << std::endl;

    isDone = true;

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


int main(int argc, char *argv[]) {
#if 0
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
    while (i < 300) {
        i++;

//        qt1_model.m_engine_ptr.get()->


        QMetaObject::invokeMethod(qt1_model.m_engine_ptr.get()->rootObjects()[0], "readValues", Qt::DirectConnection,
                                  Q_ARG(QVariant, QVariant::fromValue(list))
        );
        std::this_thread::sleep_for(std::chrono::seconds(1));

    }


    std::cout << "exit" << std::endl;
    return 0;

    //============
    GuiThread gt;
    std::shared_ptr <CppClass> caller_ptr;
    std::shared_ptr <QQmlApplicationEngine> engine_ptr;

    std::thread t(gt, std::ref(caller_ptr), std::ref(engine_ptr));

    QString ret;


    while (i < 300) {

        if (caller_ptr.use_count() == 0 || engine_ptr.use_count() == 0) {

            std::cout << "main  trig: skip " << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            continue;
        }

        (*engine_ptr).rootContext()->setContextProperty("CppClass2", caller_ptr.get());


        caller_ptr.get()->trig();
        std::cout << "main  trig: " << std::endl;
        std::cout << "caller get msg: " << caller_ptr.get()->m_msg << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    t.join();


}