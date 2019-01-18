//
// Created by waxz on 19-1-16.
//

#ifndef DEMO_QT_UTIL_H
#define DEMO_QT_UTIL_H
// qt
#include <QQmlApplicationEngine>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QQmlContext>
#include <QtQml>

// std
#include <thread>
#include <chrono>
#include <memory>
#include <iostream>


namespace qt_util {

    // qt thread model
    // as middleware between qt thread and main thread
    // shared qt resource initialise in qt thread
    // this object should sent to qt thread
    class GuiThreadModelBase {
    private:
        std::thread m_thread;
        QUrl &m_url;

        bool isDone;

    public:
        std::shared_ptr<QQmlApplicationEngine> m_engine_ptr;


    public:
        explicit GuiThreadModelBase(QUrl &url) : m_url(url), isDone(false) {};

        // create qt resource in qt thread
        void initContext();

        // register extension
        virtual void registerExtension() = 0;

        bool initDone();

    public:
        // block util all qt resource created ok
        void wait();
    };


    // a thread container for qt gui
    // initialise all qt resource in this thread
    class MQtThread {
        QUrl &m_url;
    public:
        explicit MQtThread(QUrl &url) : m_url(url) {};

        void operator()(GuiThreadModelBase &guiModel);
    };


}
#endif //DEMO_QT_UTIL_H
