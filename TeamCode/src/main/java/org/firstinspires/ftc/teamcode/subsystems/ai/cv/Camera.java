package org.firstinspires.ftc.teamcode.subsystems.ai.cv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {

    public OpenCvCamera camera;

    public Camera(HardwareMap hardwareMap, String name) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, name), cameraMonitorViewId);
        camera.setPipeline(new Pipeline(camera));

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            /**
             * Called if the camera was successfully opened
             */
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 800, OpenCvCameraRotation.UPRIGHT);
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

    public double getFPS() {
        return (double)camera.getFps();
    }

    public int getMaxFPS() {
        return camera.getCurrentPipelineMaxFps();
    }
}
