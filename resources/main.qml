import QtQuick 2.5
import QtQuick.Window 2.2
 import QtQuick.Controls 1.4
//import io.backend 1.0
//import QtQuick 2.12
import QtQuick.Layouts 1.2
import QtQuick.Controls.Styles 1.4
import "."
import "lodash.js" as L
import myextension 1.0
Window {

    id: rootWindow
    width: 800
    height: 600
    visible: true

    // example: qml function called from c++
    function readValues(anArray) {
        for (var i=0; i<anArray.length; i++){
            //console.log("Array item:", anArray[i])
        }
    }

    // example: c++ data type used in qml
//    BackEnd{
//        id: backend;
//    }




    //################ text
    Rectangle{
        id: params_b;
        anchors.top: parent.top;
        anchors.left: parent.left;
        height:120;
        width:537
        color:"white";
        anchors.leftMargin: 0
        anchors.topMargin: 0




        TextArea {
            id: bParam;
            readOnly: false;
            text:qsTr("9,1,1,1,9,1");
        
            anchors.right: parent.right;
            
            height: 30;
            anchors.rightMargin: 0
            anchors.leftMargin: 42
            //anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top;

            anchors.left: parent.left;

            anchors.topMargin: 3
            //anchors.leftMargin: 5
            //anchors.rightMargin: 5
            anchors.bottomMargin: 2

    }
    TextArea {
        id: lrParam;
        readOnly: false;
        text:qsTr("0.001");
    
        //anchors.right: parent.right;
        
        height: 30;
        width: 80;

        //anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: bParam.bottom;

        anchors.left: parent.left;

        anchors.topMargin: 2
        //anchors.leftMargin: 5
        //anchors.rightMargin: 5
        anchors.bottomMargin: 2

    }
    TextArea {
        id: stepParam;
        readOnly: false;
        text:qsTr("200");
    
        //anchors.right: lrParam.right;
        
        height: 30;
        width: 80;
        
        //anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: bParam.bottom;

        anchors.left: lrParam.right;

        anchors.topMargin: 2
        //anchors.leftMargin: 5
        //anchors.rightMargin: 5
        anchors.bottomMargin: 2

    }


        Button{
            id: load_param;
            text: qsTr("Load Param");
            anchors.top: bParam.bottom;
            anchors.topMargin: 5
           anchors.left: stepParam.right;

            height:30;
            onClicked:{
                console.log("B param text: ", bParam.text);
                var data = bParam.text.split(",");
                console.log("B param: ", data);
                var row = 2;
                var col = 3;
                var norm = 1;
                var new_data = "";
                if (data.length == 6){
                    for (var i = 0; i < row; i++){
                            var row_sum = 0.0;
                            for (var j = 0; j < col; j++){
                                var v1 = parseFloat(data[i*col + j]);
                                //console.log("-- [", i, j,"]: ", v1);
                                CppClass.setData(i, j, v1);
                                row_sum += v1;
                            }
/*
          for (var j = 0; j < col; j++){
                                var v1 = parseFloat(data[i*col + j])/row_sum;
                                //console.log("-- [", i, j,"]: ", v1);
                                CppClass.setData(i, j, v1);
                                new_data += v1.toString() + (i==1 & j == 2) ? "" : ",";
                            }
                            //console.log("-- row: ", i, ": ", row_sum);

*/
                  

                        }
                        //bParam.text = qsTr(new_data);

                }

                CppClass.setDict(qsTr("lr"),parseFloat(lrParam.text));
                CppClass.setDict(qsTr("step"),parseFloat(stepParam.text));
                CppClass.setDataDone();

            }


        }
        

// Button

        Button{
            id: gen_data;
            text: qsTr("Gen Data");
            anchors.top: bParam.bottom;
            anchors.topMargin: 5;
            anchors.left: load_param.right;
           
           //onClicked
            onClicked:{
               console.log("generate sample data");
            CppClass.genData(dataPattern.text);
           }


           //end
           
           
        }


        TextArea {
            id: dataPattern;
            readOnly: false;
            //text:qsTr("stepParam");
        
            //anchors.right: lrParam.right;
            
            //height: 30;
            //width: 80;
            
            //anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: stepParam.bottom;
            anchors.bottom: parent.bottom;

            anchors.left: parent.left;
            anchors.right: parent.right;

            anchors.topMargin: 2
            //anchors.leftMargin: 5
            //anchors.rightMargin: 5
            anchors.bottomMargin: 2

        }

    Text {
        id: text1
        x: 16
        y: 9
        text: qsTr("B")
        font.pixelSize: 12
    }




    }

// Rectangle for each model
Rectangle{
    id:model1;
    anchors.left: parent.left;
    anchors.top: params_b.bottom;
    anchors.bottomMargin: 2;

    color:"blue";
    height: 50;
    width: 537
    Text{
        width:50;
        id:model1_name;
        text:qsTr("load mdoel1")
        anchors.top: parent.top;
        anchors.left: parent.left;
        anchors.bottom: parent.bottom;

        MouseArea {
                id: model1_mousearea
                anchors.fill: parent
                onClicked: {
                    console.log("model1 clicked");
                    var data = bParam.text + "|" + lrParam.text + "|" +stepParam.text;
                    model1_param.text = qsTr(data);
                    CppClass.setSig(11);
                }
            }

    }
    TextArea {
        // display: B, lr, cache_step,
        id: model1_param;
        readOnly: false;
        //anchors.fill: parent;
        
        height: 50;
        anchors.rightMargin: 0
        //anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top;
        anchors.right: parent.right;

        anchors.left: model1_name.right;

        anchors.topMargin: 0
        anchors.leftMargin: 38
        //anchors.rightMargin: 5;
        anchors.bottomMargin: 2;


    }


}
Rectangle{
    id:model2;
    anchors.left: parent.left;
    anchors.top: model1.bottom;
    anchors.bottomMargin: 2;

    color:"red";
    height: 50;
    width: 537
    Text{
        width:50;

        id:model2_name;
        text:qsTr("load model2")
        anchors.top: parent.top;
        anchors.left: parent.left;
        anchors.bottom: parent.bottom;

        MouseArea {
                id: model2_mousearea
                anchors.fill: parent
                onClicked: {
                    console.log("model2 clicked");
                    var data = bParam.text + "|" + lrParam.text + "|" +stepParam.text;

                    model2_param.text = qsTr(data);
                    CppClass.setSig(21);
                
                }
            }
    }
    TextArea {
        // display: B, lr, cache_step,
        id: model2_param;
        readOnly: false;
        anchors.right: parent.right;
        
        height: 50;
        anchors.rightMargin: 0
        //anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top;

        anchors.left: model2_name.right;

        anchors.topMargin: 0
        anchors.leftMargin: 36
        //anchors.rightMargin: 5;
        anchors.bottomMargin: 2;


    }


}






    Rectangle{
        id:commands;
        anchors.right: parent.right;
        anchors.top: parent.top;
        width:100;
        height:200;



Rectangle{
            id:start_pause_rect;
            anchors.top: parent.top;
            anchors.topMargin: 5;
            height:100;
            width:100;
            color:"blue";


        Button{
            id: start_pause;
            //anchors.fill: parent;
            anchors.top : parent.top;
            height:25;
            anchors.topMargin: 3;
            anchors.leftMargin: 3;
            anchors.rightMargin: 3;
            anchors.bottomMargin: 3;

            text: qsTr("start/pause");
            property var state;
            state:false;

            onClicked:{
                state = !state;
                console.log((state)?"start : ":"pause", state);
                start_pause_rect.color = (state)?"green" :"red";
                CppClass.setSig((state)?1:0);
                CppClass.updateMap("state", text);
                var mm = [[1.3,4.5,6,7],[1,2,3,4]];
                CppClass.updateMap("mm", mm);

            }
        }
        Button{
            id: reset;
            anchors.top : start_pause.bottom;
            height:25;
            //anchors.bottom : parent.bottom;
            anchors.topMargin: 3;
            anchors.leftMargin: 3;
            anchors.rightMargin: 3;
            anchors.bottomMargin: 3;

            text: qsTr("reset");

            onClicked:{
                
                console.log("reset");
                start_pause_rect.color = (state)?"green" :"red";
                CppClass.setSig(2);

            }
        }
               Button{
                    id: printparam;
                    anchors.top : reset.bottom;
                    height:25;
                    //anchors.bottom : parent.bottom;
                    anchors.topMargin: 3;
                    anchors.leftMargin: 3;
                    anchors.rightMargin: 3;
                    anchors.bottomMargin: 3;

                    text: qsTr("printparam");

                    onClicked:{

                        console.log("printparam");
                        //start_pause_rect.color = (state)?"green" :"red";
                        CppClass.setSig(3);

                    }
                }

}

    }
    DataGrid{
         id: grid;
         header_text: "Params";
         anchors.top: commands.bottom;
         anchors.right : parent.right;
         data_matrix : [[1,2,3],[3,4,5]];

    }

Image{
        id:img;

        anchors.top: commands.bottom;
        anchors.right : parent.right;
        width: 460;
        height : 680;
    }
    Connections{
        target: CodeImage
        onCallQmlRefeshImg:{
            img.source = "image://CodeImg/"+ Math.random()
        }
    }




    
}
