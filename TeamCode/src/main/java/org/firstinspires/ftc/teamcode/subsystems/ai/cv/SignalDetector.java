package org.firstinspires.ftc.teamcode.subsystems.ai.cv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Constants;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;

import java.io.IOException;

public class SignalDetector {

    private TensorImageClassifier tfic;
    private VuforiaLocalizer vuforia;

    public SignalDetector(HardwareMap hardwareMap, Telemetry telemetry) throws IOException {

        // Create TFICBuilder
        TFICBuilder builder = new TFICBuilder(
                hardwareMap,
                Constants.OD_RES_MAIN_TFLITE,
                "Side 1", "Side 2", "Side 3"
        );
        // Additional Options go here

        tfic = builder.build();
    }
}
