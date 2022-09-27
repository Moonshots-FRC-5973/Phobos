package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test Suite", group="TeleOp")
public class TestSuite extends OpMode {
    private DcMotor leftMotorOne;
    private DcMotor leftMotorTwo;
    private DcMotor rightMotorOne;
    private DcMotor rightMotorTwo;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        leftMotorOne = hardwareMap.get(DcMotor.class, "left_drive_one");
        leftMotorTwo = hardwareMap.get(DcMotor.class, "left_drive_two");
        //rightMotorOne = hardwareMap.get(DcMotor.class, "right_drive_one");
        //rightMotorTwo = hardwareMap.get(DcMotor.class, "right_drive_two");
        telemetry.addData("Status", "Initialized.");
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        leftMotorOne.setPower(Range.clip(
                gamepad1.left_stick_y + gamepad1.left_stick_x,
                -0.75,
                0.75
        ));
        leftMotorTwo.setPower(Range.clip(
                gamepad1.left_stick_y - gamepad1.left_stick_x,
                -0.75,
                0.75
        ));
    }

    @Override
    public void stop() {
        leftMotorOne.setPower(0.0d);
        leftMotorTwo.setPower(0.0d);
    }
}
