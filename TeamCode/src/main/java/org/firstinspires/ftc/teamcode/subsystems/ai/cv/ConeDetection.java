package org.firstinspires.ftc.teamcode.subsystems.ai.cv;

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
    private FtcDashboard ftcDashboard;
    private Telemetry telemetry;

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public ConeDetection(HardwareMap hardwareMap) {
        // Init FTCDashboard
        ftcDashboard = FtcDashboard.getInstance();
        if(ftcDashboard == null) {
            throw new NullPointerException("FTCDashboard instance was null after creation");
        }
        telemetry = ftcDashboard.getTelemetry();
        if(telemetry == null) {
            throw new NullPointerException("FTCDashboard Telemetry was null after creation");
        }

        //Init Vuforia (required apparently? idk)
        VuforiaLocalizer.Parameters vuParams = new VuforiaLocalizer.Parameters(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        vuParams.vuforiaLicenseKey = Constants.OD_VUFORIA_KEY;
        vuParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(vuParams);

        // Put the camera on the FTC Dashboard
        ftcDashboard.startCameraStream(vuforia, 30);

        //Init TFLite
        TFObjectDetector.Parameters tfliteParams = new TFObjectDetector.Parameters();
        tfliteParams.minResultConfidence = 0.75f;
        tfliteParams.isModelTensorFlow2 = true;
        tfliteParams.inputSize = 300;
        tflite = ClassFactory.getInstance().createTFObjectDetector(tfliteParams, vuforia);

        try {
            tflite.loadModelFromAsset(Constants.OD_RES_MAIN_TFLITE, Constants.OD_LABELS);
        } catch(Exception e) {
            try {
                tflite.loadModelFromFile(Constants.OD_RES_MAIN_TFLITE, Constants.OD_LABELS);
            } catch(Exception exception) {
                telemetry.addData("TFLite", "No Model Loaded");
            }
        }

        if(tflite == null) {
            throw new ExceptionInInitializerError("TFLite object was null after creation");
        }
        tflite.activate();
        tflite.setZoom(1.0, 16.0/9.0);
    }

    public void update() {
        telemetry.addData("TFLite", this.toString());
        telemetry.update();
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
        for(Recognition recognition : recognitionList) {if(recognition.getLabel() == "Blue Cone") {return true}}
        return false;
    }
}
