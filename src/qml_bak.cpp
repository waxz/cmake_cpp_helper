#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QtQml>
#include <QQuickView>
#include <thread>
#include <chrono>
#include "backend.h"
#include <iostream>
#include <memory>
#include <thread>                   // thread
#include <memory>                   // shared_ptr
#include <functional>               // bind
#include <mutex>                    // mutex, lock
#include <condition_variable>       // condition_variable_any
#include <chrono>                   // time and sleep
#include <map>                      // map

#include <boost/signals2.hpp>       // signals2

#if 1


class SignalCaller {
public:
    typedef boost::signals2::signal<void(std::string)> signal_t;

//    typedef T signal_t;
private:
    signal_t m_sig;
public:

    explicit SignalCaller() {

    }

    boost::signals2::connection connect(const signal_t::slot_type &sub) {
        return m_sig.connect(sub);
    }

    void setText(std::string text) {
        m_sig(text);

        return;
    };

};


struct InputArgs {
    int argc;
    char **argv;
};
/*qt thread
 * must create all instance in this thread
 * return shared_ptr to main thread
 */
typedef std::shared_ptr <QQmlApplicationEngine> QQmlApplicationEngine_Ptr;

struct GuiEntry {
    QGuiApplication m_app;
    QQmlApplicationEngine m_engine;
    QQmlApplicationEngine_Ptr m_engine_ptr;
    SignalCaller &m_sig;

    GuiEntry(std::string path, InputArgs args, QQmlApplicationEngine_Ptr &engine_ptr, SignalCaller &sig) :
            m_app(args.argc, args.argv),
            m_engine(),
            m_engine_ptr(std::make_shared<QQmlApplicationEngine>()),
            m_sig(sig) {
        engine_ptr = m_engine_ptr;
        qmlRegisterType<BackEnd>("io.backend", 1, 0, "BackEnd");

        (*m_engine_ptr).load(QUrl(QString::fromStdString(path)));

        m_sig.connect(boost::bind(&GuiEntry::setText, this, _1));

    }

    void setText(std::string text) {

        QObject *object = this->m_engine_ptr->rootObjects()[0];
        QObject *mrect = object->findChild<QObject *>("mRectangle");
        mrect->setProperty("color", "red");
        QObject *box = object->findChild<QObject *>("textInput");

        box->setProperty("text", text.c_str());
        std::cout << "Sig SetText" << text << std::endl;

    }

    void run() {
        std::cout << "app run" << std::endl;

        m_app.exec();

    }

    void operator()() {

        std::cout << "start thread" << std::endl;
        std::cout << "app run" << std::endl;

        m_app.exec();

    }
};

class GuiThread {
protected:
    std::string m_path;
    InputArgs m_args;
    SignalCaller m_sig;


public:
    QQmlApplicationEngine_Ptr m_engine_ptr;
public:
    GuiThread(std::string path, InputArgs args) :
            m_path(path),
            m_args(args),
            m_sig() {

    }

    QQmlApplicationEngine_Ptr &engine() {
        while (this->m_engine_ptr.use_count() == 0) {
            std::cout << "ptr empty: " << this->m_engine_ptr.use_count() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));

        }
        return m_engine_ptr;
    }

    void setText(std::string text) {
        m_sig.setText(text);
    }

    void operator()() {
        GuiEntry entry(m_path, m_args, m_engine_ptr, m_sig);
        std::cout << "debug: entry.m_engine_ptr" << entry.m_engine_ptr.use_count() << std::endl;
//        this->m_engine_ptr = entry.m_engine_ptr;
        std::cout << "debug: m_engine_ptr" << this->m_engine_ptr.use_count() << std::endl;

#if 1
        QObject *object = this->m_engine_ptr->rootObjects()[0];
        QObject *mrect = object->findChild<QObject *>("mRectangle");
        mrect->setProperty("color", "red");
        QObject *box = object->findChild<QObject *>("textInput");
#endif

        box->setProperty("text", "weiwei");
        entry.run();
    }
};

#endif

void f1(std::shared_ptr<int> &p) {
    std::cout << "thread p: " << p.get() << std::endl;

    std::shared_ptr<int> p2 = std::make_shared<int>(89);
    p = p2;
    std::cout << "thread p.use_count " << p.use_count() << std::endl;

}


int main(int argc, char **argv) {


    InputArgs args{argc, argv};
    auto url1 = QUrl::fromLocalFile("/home/waxz/dev/notebook/cmake_helper/resources/main.qml");
#if 0

    std::shared_ptr<int> p1;
    std::cout << "p1.use_count " << p1.use_count() << std::endl;

    std::thread t1(f1,std::ref(p1));
    std::thread t2
            ([&]{


                std::shared_ptr<int> p3 = std::make_shared<int>(9);

                 return 1;
             }
            );



    t2.join();
    t1.join();
    std::cout << "p1.use_count " << p1.use_count() << std::endl;

    std::cout << "p1: " << p1.get() << std::endl;
    return 0;


#endif

    // relative resource or binary resource
    std::string path = "qrc:/main.qml";


//    auto url2 = QUrl(QStringLiteral("qrc:/main.qml"));
//    auto url3 = QUrl(QString::fromStdString("qrc:/main.qml"));
//
//    QQmlApplicationEngine engine;
//
//    engine.load(QUrl(url1));
//
//    QObject *object = engine.rootObjects()[0];
//    QObject *mrect = object->findChild<QObject*>("mRectangle");
//    mrect->setProperty("color", "red");
//    QObject *box = object->findChild<QObject*>("textInput");
//
//    box->setProperty("text","weiwei");

    GuiThread gt(path, args);

    std::thread t(std::ref(gt));
#if 0
    while (1){
        std::cout << " gt.m_engine_ptr "<< gt.m_engine_ptr.use_count() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

    }
#endif
    gt.engine()->rootObjects()[0];
#if 1
    QObject *object = gt.engine()->rootObjects()[0];
    QObject *mrect = object->findChild<QObject *>("mRectangle");
    mrect->setProperty("color", "red");
    QObject *box = object->findChild<QObject *>("textInput");
#endif

    box->setProperty("text", "weiwei");

    for (int i = 0; i < 100; i++) {
        box->setProperty("text", i);
        gt.setText("oop");
        std::cout << "update text: " << i << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    t.join();



//    n.sprintf("ss : %s");


//    if (engine.rootObjects().isEmpty())
//        return -1;

//    QQmlEngine engine2;
////    QQmlComponent component(&engine2,QUrl(QStringLiteral("qrc:/main.qml")));
//    QQmlComponent component(&engine2,url1);
//
//    QObject *object = component.create();
//    QObject *rect = object->findChild<QObject*>("textInput");
//    rect->setProperty("text", "red");
//
//
//    delete object;
    return 1;
}