import QtQuick 2.3
import QtQuick.Controls 1.0
import io.backend 1.0

ApplicationWindow {
    id: root
    width: 300
    height: 480
    visible: true
    Rectangle
    {
        id: mrect
        objectName: "mRectangle"
        x: 10
        y: 20
        height: 10
        width: 10
    }
    BackEnd {
        id: backend
    }
    MouseArea {
        id: loginMouseArea
        hoverEnabled: true
        anchors.fill: parent

        onClicked:{
            console.log("Login pressed" ,backend.userName);
            textInput.text = 666;
            }
    }

    Rectangle
    {

        anchors.centerIn: parent


        TextField {
            objectName: "textInput"
            id: textInput
            text: backend.userName
            placeholderText: qsTr("User name")
            anchors.centerIn: parent

            onTextChanged: {
            backend.userName = text;
            //backend.setUserName(8888);

            }
        }

    }



}