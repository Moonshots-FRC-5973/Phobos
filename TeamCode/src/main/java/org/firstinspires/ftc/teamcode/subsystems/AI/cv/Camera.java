package org.firstinspires.ftc.teamcode.subsystems.AI.cv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Camera {

    OpenCvCamera camera;

    public Camera(HardwareMap hardwareMap, String name) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, name));
        camera.setPipeline(new Pipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            /**
             * Called if the camera was successfully opened
             */
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            /**
             * Called if there was an error opening the camera
             *
             * @param errorCode reason for failure
             */
            @Override
            public void onError(int errorCode) {
                throw new RuntimeException();
            }
        });
    }
}
