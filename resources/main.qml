import QtQuick 2.3
import QtQuick.Window 2.2

// main window

Window {

    visible: true
    MouseArea {
        anchors.fill: parent
        onClicked: {
            	Qt.quit();
        }
    }
    Text {
        id: keyView;
        font.bold: true;
        font.pixelSize: 24;
        text: qsTr("text");
        anchors.centerIn: parent;
    }

	Image {
        source: "pics/cpp.png"
    }



}

