package org.firstinspires.ftc.teamcode.subsystems.ai.cv;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    private OpenCvCamera camera;
    public Pipeline(OpenCvCamera camera) {
        super();
        this.camera = camera;
    }
    private boolean isViewportPaused = false;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        return input;
    }
    @Override
    public void onViewportTapped() {
        isViewportPaused = !isViewportPaused;

        if(isViewportPaused)
        {
            camera.pauseViewport();
        }
        else
        {
            camera.resumeViewport();
        }
    }
}
