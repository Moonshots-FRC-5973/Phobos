package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive extends Drivetrain {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    public MecanumDrive(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive_front");
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_drive_back");
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_drive_back");
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double forward, double strafe, double turn) {
        if(Math.abs(forward) <= Constants.DRIVE_INPUT_THRESHOLD) {
            forward = 0.0d;
        }

        if(Math.abs(strafe) <= Constants.DRIVE_INPUT_THRESHOLD) {
            strafe = 0.0d;
        }

        if (Math.abs(turn) >= Constants.DRIVE_INPUT_THRESHOLD) {
            drive(-turn, turn, turn, turn);
            return;
        }

        drive(
              forward - strafe,
              forward - strafe,
              -forward + strafe,
              -forward + strafe
        );
    }

    public void drive(double m1, double m2, double m3, double m4) {
        if(telemetry != null) {
            telemetry.addData("m1", m1);
            telemetry.addData("m2", m2);
            telemetry.addData("m3", m3);
            telemetry.addData("m4", m4);
        }
        leftFrontDrive.setPower(Range.clip(m1, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftBackDrive.setPower(Range.clip(m2, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightFrontDrive.setPower(Range.clip(m3, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightBackDrive.setPower(Range.clip(m4, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
    }

    @Override
    public void resetWheels() {
        drive(
                -leftFrontDrive.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                -leftBackDrive.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                -rightFrontDrive.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                -rightBackDrive.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV
        );
    }
}
