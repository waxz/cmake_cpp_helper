#include "backend.h"

BackEnd::BackEnd(QObject *parent) :
        QObject(parent) {
}

QString BackEnd::userName() const {
    std::cout << "call userName" << std::endl;
    return m_userName;
}

void BackEnd::setUserName(const QString &userName) {
    std::cout << "call setUserName" << std::endl;

    if (userName == m_userName)
        return;

    m_userName = userName;
    emit userNameChanged();
}

void BackEnd::fuck(const QString &userName) {

    std::cout << "fuck: " << userName.toStdString() << std::endl;
}


QString BackEnd::createBackEnd() {
//    BackEnd nb;
//    this->setUserName(QString::fromStdString("67890"));
    return QString::fromStdString("cpp!!");
}


//===============
CppClass::CppClass(QObject *parent) : QObject(parent) {
    m_matrix.setOnes();
    m_sig = 0;
    m_sample_data.clear();
}

void CppClass::genData(QString pattern) {
    auto data = pattern.split(QStringLiteral(","));
    std::cout << "get pattern: " << std::endl;
    m_sample_data.clear();

    int batch = atoi(data[0].toStdString().c_str());

    for (int b = 0; b < batch; b++) {
        for (int i = 1; i < data.size(); i++) {
            int d = atoi(data[i].toStdString().c_str());
            if (d > 2) {
                for (int j = 0; j < d; j++) {
                    m_sample_data.push_back(atoi(data[i + 1].toStdString().c_str()));

                }
            } else {
                m_sample_data.push_back(d);

            }
//        std::cout <<  << std::endl;
        }
    }

}

CppClass::~CppClass() {

}

void CppClass::getCurrentTime() {
    emit timeUpdate(QDateTime::currentDateTime().toString("ddd dd MMMM yyyy hh:mm:ss.zzz"));
}

void CppClass::trig() {
    this->getCurrentTime();
    std::cout << "CppClass trig: " << std::endl;

}

void CppClass::setData(int row, int col, float d) {
    std::cout << "get data from qml: " << row << "," << col << "," << d << std::endl;

//    m_data[row][col] = d;
    m_matrix(row, col) = d;
}

void CppClass::setDict(QString k, float v) {
    std::string kn = k.toStdString();
    std::cout << "get data from qml: " << kn << ", " << v << std::endl;

    m_dict[kn] = v;

}

void CppClass::setDataDone() {
    for (int i = 0; i < m_matrix.rows(); i++) {
        m_matrix.row(i) /= m_matrix.row(i).sum();
    }
    std::cout << "matrix: " << m_matrix << std::endl;
}

void CppClass::setSig(int sig) {
    std::cout << "get sig from qml: " << sig << std::endl;

    m_sig = sig;
}

void CppClass::updateMap(QString k, QVariant v) {
    //QImage tmpImg =v.toMap().value("Img").value<QImage>();//这句话拿到传入函数的图
    m_map[k.toStdString().c_str()] = v;
    std::cout << "===== update qmap " << k.toStdString().c_str() << std::endl;
    qDebug() << m_map[k.toStdString().c_str()].toString();

}

void CppClass::sendMessage(const QString &msg, QQuickItem *textArea) {
    QQuickItem item();

    textArea->setHeight(90);
    QTextDocument *textDocument = textArea->property("textDocument").value<QQuickTextDocument *>()->textDocument();

    textDocument->setHtml(textDocument->toHtml() + "\n<b>Text sent to Cpp side:</b> <i>" + msg + "</i>");

    std::cout << "sendMessage: " << msg.toStdString() << std::endl;
    m_msg = msg.toStdString();
}

void CppClass::sendBackEnd(BackEnd *backEnd) {

    std::cout << "CppClass get back end: " << backEnd->userName().toStdString() << std::endl;
}


ImageItem::ImageItem(QQuickItem *parent) : QQuickPaintedItem(parent) {
    this->current_image = QImage(":/images/background.jpg");
}

void ImageItem::paint(QPainter *painter) {
    QRectF bounding_rect = boundingRect();
    QImage scaled = this->current_image.scaledToHeight(bounding_rect.height());
    QPointF center = bounding_rect.center() - scaled.rect().center();

    if (center.x() < 0)
        center.setX(0);
    if (center.y() < 0)
        center.setY(0);
    painter->drawImage(center, scaled);
}

QImage ImageItem::image() const {
    return this->current_image;
}

void ImageItem::setImage(const QImage &image) {
    std::cout << "ImageItem get image " << std::endl;
    this->current_image = image;
    update();
}
//################


LiveImageProvider::LiveImageProvider() : QQuickImageProvider(QQuickImageProvider::Image) {
    this->no_image = QImage(":/images/background.jpg");
    this->blockSignals(false);
}

/**
 * @brief Delivers image. The id is not used.
 * @param id The id is the requested image source, with the "image:" scheme and provider identifier removed.
 * For example, if the image source was "image://myprovider/icons/home", the given id would be "icons/home".
 * @param size In all cases, size must be set to the original size of the image. This is used to set the
 * width and height of the relevant Image if these values have not been set explicitly.
 * @param requestedSize The requestedSize corresponds to the Image::sourceSize requested by an Image item.
 * If requestedSize is a valid size, the image returned should be of that size.
 * @return
 */
QImage LiveImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize) {
    QImage result = this->image;

    if (result.isNull()) {
        result = this->no_image;
    }

    if (size) {
        *size = result.size();
    }

    if (requestedSize.width() > 0 && requestedSize.height() > 0) {
        result = result.scaled(requestedSize.width(), requestedSize.height(), Qt::KeepAspectRatio);
    }

    return result;
}

/**
 * @brief Update of the current image.
 * @param image The new image.
 */
void LiveImageProvider::updateImage(const QImage &image) {
    if (this->image != image) {
        this->image = image;
        emit imageChanged();
    }
}

ImageProvider::ImageProvider() : QQuickImageProvider(QQuickImageProvider::Image) {

}

QImage ImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize) {
    return this->img;
}

QPixmap ImageProvider::requestPixmap(const QString &id, QSize *size, const QSize &requestedSize) {
    return QPixmap::fromImage(this->img);
}

ShowImage::ShowImage(QObject *parent) : QObject(parent) {
    m_pImgProvider = new ImageProvider();
}

void ShowImage::setImage(QImage image) {
    m_pImgProvider->img = image;
    emit callQmlRefeshImg();
}


// qt gui thread model


void GuiThreadModel::registerExtension() {

    m_caller_ptr = std::make_shared<CppClass>();

    m_img_ptr = std::make_shared<LiveImageProvider>();

    m_showimg_ptr = std::make_shared<ShowImage>();


    std::cout << "========qmlRegisterType========" << std::endl;

    qmlRegisterType<BackEnd>("io.backend", 1, 0, "BackEnd");
    qmlRegisterType<ImageItem>("myextension", 1, 0, "ImageItem");


    std::cout << "========setContextProperty========" << std::endl;

    (*m_engine_ptr).rootContext()->setContextProperty("CppClass", m_caller_ptr.get());
    (*m_engine_ptr).rootContext()->setContextProperty("liveImageProvider", m_img_ptr.get());
    (*m_engine_ptr).addImageProvider("live", m_img_ptr.get());

    std::cout << "========CodeImage========" << std::endl;

    (*m_engine_ptr).rootContext()->setContextProperty("CodeImage", m_showimg_ptr.get());
    (*m_engine_ptr).addImageProvider(QLatin1String("CodeImg"), m_showimg_ptr.get()->m_pImgProvider);
    std::cout << "========CodeImage done========" << std::endl;

}