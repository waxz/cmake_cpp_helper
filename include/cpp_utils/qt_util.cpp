//
// Created by waxz on 19-1-16.
//

#include "qt_util.h"


namespace qt_util {
    // ==========================
// GuiThreadModelBase


// create qt resource
// run in qt thread
    void GuiThreadModelBase::initContext() {

        std::cout << "========initContext========" << std::endl;

        // create gui engine
        m_engine_ptr = std::make_shared<QQmlApplicationEngine>();

        // register extension
        registerExtension();

        // load qml url
        (*m_engine_ptr).load(m_url);
        std::cout << "========load url========" << std::endl;

        // check qml load
        isDone = !m_engine_ptr.get()->rootObjects().empty();

        // there may be sytax error in qml file
        if (!isDone) {
            throw std::logic_error((m_url.toString().toStdString() + " load fail").c_str());
        }
        std::cout << "========initContext DONE ========" << std::endl;

    }

// check init result
    bool GuiThreadModelBase::initDone() {

        return isDone;
    }

// create qt thread
// and block util resource created ok
    void GuiThreadModelBase::wait() {


        MQtThread qt_thread(m_url);

        m_thread = std::thread(qt_thread, std::ref(*this));
        std::cout << "QtGuiThread create thread ok " << std::endl;

        m_thread.detach();
        //m_thread.join();

        // block until all gui created ok


        while (!initDone()) {

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << "QtGuiThread initDone" << std::endl;

    }


//====================
// MQtThread
    void MQtThread::operator()(GuiThreadModelBase &guiModel) {
        int argc;
        char **argv{};
        QGuiApplication app(argc, argv);
        guiModel.initContext();
        app.exec();
    }


}
