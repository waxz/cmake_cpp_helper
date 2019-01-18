#ifndef BACKEND_H
#define BACKEND_H

#include <Eigen/Eigen>

#include <cpp_utils/qt_util.h>
#include <QObject>
#include <QString>
#include <QObject>
#include <QQuickImageProvider>
#include<QImage>
#include<QTcpSocket>
#include <QTcpServer>
#include<QBuffer>

#include <QDateTime>
//### New Code ###
#include <QQuickPaintedItem>
#include <QQuickItem>
#include <QPainter>
#include <QImage>

#include <QQuickItem>
#include <QQuickTextDocument>
#include <QTextDocument>
#include <iostream>
#include <memory>
#include <thread>                   // thread
#include <memory>                   // shared_ptr
#include <functional>               // bind
#include <mutex>                    // mutex, lock
#include <condition_variable>       // condition_variable_any
#include <chrono>                   // time and sleep
#include <map>                      // map





class BackEnd : public QObject {
Q_OBJECT
    Q_PROPERTY(QString userName
                       READ
                       userName
                       WRITE
                       setUserName
                       NOTIFY
                       userNameChanged)

public:
    explicit BackEnd(QObject *parent = nullptr);

    QString userName() const;

    Q_INVOKABLE void fuck(const QString &userName);

    void setUserName(const QString &userName);

    Q_INVOKABLE QString createBackEnd();
signals:

    void userNameChanged();

private:
    QString m_userName;
};

//################

class CppClass : public QObject {
    /*The Q_OBJECT macro is expanded by the preprocessor to declare several member functions that are implemented by the moc; if you get compiler errors along the lines of "undefined reference to vtable for LcdNumber", you have probably forgotten to run the moc or to include the moc output in the link command.
     * */

Q_OBJECT
public:
    explicit CppClass(QObject *parent = 0);

    ~CppClass();

    Q_INVOKABLE void getCurrentTime();

    void trig();

    //### New Code ###

    Q_INVOKABLE void sendMessage(const QString &msg, QQuickItem *textArea);

    //################
public:
    QVariantMap m_map;
    std::string m_msg;
    float m_data[2][3];
    int m_sig;
    std::map<std::string, float> m_dict;
    Eigen::Matrix<float, 2, 3> m_matrix;
    Q_INVOKABLE void setDict(QString k, float v);

    Q_INVOKABLE void genData(QString pattern);

    Q_INVOKABLE void setData(int row, int col, float d);

    Q_INVOKABLE void setDataDone();

    Q_INVOKABLE void setSig(int sig);

    Q_INVOKABLE void updateMap(QString k, QVariant v);

    std::vector<int> m_sample_data;

    Q_INVOKABLE void sendBackEnd(BackEnd *backEnd);

signals:

    void timeUpdate(QString currentTime);

public slots:
};


class ImageItem : public QQuickPaintedItem {
Q_OBJECT
    Q_PROPERTY(QImage image
                       READ
                       image
                       WRITE
                       setImage
                       NOTIFY
                       imageChanged)
public:
    ImageItem(QQuickItem *parent = nullptr);

    Q_INVOKABLE void setImage(const QImage &image);

    void paint(QPainter *painter);

    QImage image() const;

signals:

    void imageChanged();

private:
    QImage current_image;
};


class LiveImageProvider : public QObject, public QQuickImageProvider {
Q_OBJECT
public:
    LiveImageProvider();

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;

public slots:

    void updateImage(const QImage &image);

signals:

    void imageChanged();

private:
    QImage image;
    QImage no_image;
};

class ImageProvider : public QQuickImageProvider {
public:
    ImageProvider();

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize);

    QPixmap requestPixmap(const QString &id, QSize *size, const QSize &requestedSize);

    QImage img;
};

class ShowImage : public QObject {

Q_OBJECT
public:
    explicit ShowImage(QObject *parent = 0);

    ImageProvider *m_pImgProvider;
public slots:

    void setImage(QImage image);

signals:

    void callQmlRefeshImg();
};


class GuiThreadModel : public qt_util::GuiThreadModelBase {


public:

    std::shared_ptr<CppClass> m_caller_ptr;

    std::shared_ptr<LiveImageProvider> m_img_ptr;

    std::shared_ptr<ShowImage> m_showimg_ptr;

public:
    explicit GuiThreadModel(QUrl &url) : GuiThreadModelBase(url) {};

    void registerExtension();
};

#endif // BACKEND_H