package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ai.cv.ConeDetection;

import java.util.List;

//Test Suite Code.
@TeleOp(name="Test Suite", group="TeleOp")
public class TestSuite extends OpMode {

    private DigitalChannel limSwitch;

    @Override
    public void init() {
        limSwitch = hardwareMap.get(DigitalChannel.class, "arm_limit");
        limSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void init_loop() {
        telemetry.addData("lim", limSwitch.getState());
    }

    public void start() {

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {

    }

    @Override
    public void stop() {
    }
}
