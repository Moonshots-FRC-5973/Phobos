package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveDrive {
    private DcMotor leftMotorOne;
    private DcMotor leftMotorTwo;
    private DcMotor rightMotorOne;
    private DcMotor rightMotorTwo;

    public SwerveDrive(HardwareMap hardwareMap) {
        leftMotorOne = hardwareMap.get(DcMotor.class, "left_drive_one");
        leftMotorOne = hardwareMap.get(DcMotor.class, "left_drive_two");
    }

    public void init() {

    }

    public void stop() {
        leftMotorOne.setPower(0.0d);
        leftMotorTwo.setPower(0.0d);
    }
}
