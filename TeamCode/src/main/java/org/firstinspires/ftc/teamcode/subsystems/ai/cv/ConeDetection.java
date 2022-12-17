package org.firstinspires.ftc.teamcode.subsystems.ai.cv;

import android.service.autofill.RegexValidator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public class ConeDetection {
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tflite;

    public enum SignalStatus {
        SIDE_1,
        SIDE_2,
        SIDE_3,
        ERROR
    };

    public SignalStatus status;

    public ConeDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        //Init Vuforia (required apparently? idk)
        VuforiaLocalizer.Parameters vuParams = new VuforiaLocalizer.Parameters(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        vuParams.vuforiaLicenseKey = Constants.OD_VUFORIA_KEY;
        vuParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(vuParams);

        // Put the camera on the FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(vuforia, 30);

        //Init TFLite
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfliteParams = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfliteParams.minResultConfidence = 0.75f;
        tfliteParams.isModelTensorFlow2 = true;
        tfliteParams.inputSize = 300;
        tflite = ClassFactory.getInstance().createTFObjectDetector(tfliteParams, vuforia);

        tflite.loadModelFromFile(Constants.OD_RES_MAIN_TFLITE, Constants.OD_LABELS);
        telemetry.addData("tflite-init", "Model Loaded from file");


        if (tflite != null) tflite.activate();
        else telemetry.addData("tflite-init", "NULL");

        telemetry.addData("tflite-init", "Activated");
        tflite.setZoom(1.0, 16.0/9.0);
        telemetry.addData("tflite-init", "Zoom set");

        /*// can we get the weighted probality of recongnition... not just first one
        for(Recognition r: rList) {
            switch(r.getLabel()) {
                case "Side 1":
                    this.status = SignalStatus.SIDE_1;
                    break;
                case "Side 2":
                    this.status = SignalStatus.SIDE_2;
                    break;
                case "Side 3":
                    this.status = SignalStatus.SIDE_3;
                default:
                    this.status = SignalStatus.ERROR;
            }
            if(this.status != SignalStatus.ERROR) {
                break;
            }

        }

         */
    }

    public List<Recognition> getRecognitions() {
        return tflite.getRecognitions();
    }

    @Override
    public String toString() {
        String obj = "TFLite instance:\n\t";
        List<Recognition> recognitionList = getRecognitions();
        if(recognitionList.size() == 0) {
            obj += "No objects detected.\n";
        } else {
            obj += "Number of objects detected: " + recognitionList.size();
            for(Recognition r : recognitionList) {
                obj += "\n\t\t" + r.getLabel();
            }
        }
        return obj;
    }

    public boolean isRedConeInView() {
        List<Recognition> rList = getRecognitions();
        if(rList == null) { return false; }
        for(Recognition r : rList) { if(r.getLabel() == "Red Cone") { return true; } }
        return false;
    }
    public boolean isBlueConeInView(){
        List<Recognition> recognitionList = getRecognitions();
        if(recognitionList == null) {return false;}
        for(Recognition recognition : recognitionList) {if(recognition.getLabel() == "Blue Cone") {return true;}}
        return false;
    }


}
