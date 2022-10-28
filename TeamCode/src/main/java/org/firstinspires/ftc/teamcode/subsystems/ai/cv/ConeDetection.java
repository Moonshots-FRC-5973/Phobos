package org.firstinspires.ftc.teamcode.subsystems.ai.cv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public class ConeDetection {
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tflite;

    public ConeDetection(HardwareMap hardwareMap) {
        //Init Vuforia (required apparently? idk)
        VuforiaLocalizer.Parameters vuParams = new VuforiaLocalizer.Parameters();
        vuParams.vuforiaLicenseKey = Constants.OD_VUFORIA_KEY;
        vuParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(vuParams);

        //Init TFLite
        TFObjectDetector.Parameters tfliteParams = new TFObjectDetector.Parameters();
        int viewID = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        tfliteParams.minResultConfidence = 0.75f;
        tfliteParams.isModelTensorFlow2 = true;
        tfliteParams.inputSize = 300;
        tflite = ClassFactory.getInstance().createTFObjectDetector(tfliteParams, vuforia);
        try {
            tflite.loadModelFromAsset(Constants.OD_RES_MAIN_TFLITE, Constants.OD_LABELS);
        } catch(Exception e) {
            tflite.loadModelFromFile(Constants.OD_RES_MAIN_TFLITE, Constants.OD_LABELS);
        }

        if(tflite == null) {
            throw new ExceptionInInitializerError("TFLite object was null after creation");
        }
        tflite.activate();
        tflite.setZoom(1.0, 16.0/9.0);
    }

    public void update() {
        tflite.getUpdatedRecognitions();
    }

    public List<Recognition> getRecognitions() {
        return tflite.getRecognitions();
    }

    @Override
    public String toString() {
        String obj = "TFLite instance:\n\t";
        List<Recognition> recognitionList = getRecognitions();
        if(recognitionList == null) {
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
}
