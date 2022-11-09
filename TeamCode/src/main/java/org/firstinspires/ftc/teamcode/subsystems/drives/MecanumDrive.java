package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public class MecanumDrive {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private IMU imu;

    private Telemetry telemetry;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive_front") ;
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_drive_back") ;
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive") ;
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_back_drive") ;

        imu = new IMU(hardwareMap);
    }

    public void drive(double forward, double strafe, double turn) {
        if (Math.abs(turn)>= Constants.DRIVE_INPUT_THRESHOLD) {
            drive(turn, turn, -turn, -turn);
            return;
        }

        drive(
                forward + strafe,
                forward - strafe,
                forward - strafe,
                forward + strafe
        );
    }

    public void drive(double m1, double m2, double m3, double m4) {
        leftFrontDrive.setPower(Range.clip(m1, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftBackDrive.setPower(Range.clip(m2, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightFrontDrive.setPower(Range.clip(m3, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightBackDrive.setPower(Range.clip(m4, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
    }

    public void stop() {
        drive(0.0d, 0.0d, 0.0d, 0.0d);
    }
}
