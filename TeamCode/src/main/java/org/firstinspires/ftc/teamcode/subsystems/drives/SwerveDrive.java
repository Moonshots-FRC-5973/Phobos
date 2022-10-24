package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.sli.Vec2;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public class SwerveDrive {
    private DcMotor leftMotorOne;
    private DcMotor leftMotorTwo;
    private DcMotor rightMotorOne;
    private DcMotor rightMotorTwo;
    private double speed = 1;
    private boolean fieldCentric = true;

    private IMU imu;

    public int getLeftEncodersDifference() {
        return leftMotorOne.getCurrentPosition() - leftMotorTwo.getCurrentPosition();
    }
    public double getLeftWheelAngle() {
        double angle = (360 * (getLeftEncodersDifference() % Constants.DRIVE_ENCODER_COUNTS_PER_REV) / Constants.DRIVE_ENCODER_COUNTS_PER_REV);
        if(fieldCentric) return angle + imu.getZAngle();
        return angle;
    }
    public int getRightEncodersDifference() {
        return rightMotorOne.getCurrentPosition() - rightMotorTwo.getCurrentPosition();
    }
    public double getRightWheelAngle() {
        double angle =  (360 * (getRightEncodersDifference() % Constants.DRIVE_ENCODER_COUNTS_PER_REV) / Constants.DRIVE_ENCODER_COUNTS_PER_REV);
        if(fieldCentric) return angle + imu.getZAngle();
        return angle;
    }
    public void setSpeed(double speed) {
        this.speed = speed;
    }
    public double getSpeed() {
        return speed;
    }

    public void makeFieldCentric() {
        fieldCentric = true;
    }

    public void makeRobotCentric() {
        fieldCentric = false;
    }

    public void switchMode() {
        fieldCentric = !fieldCentric;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public SwerveDrive(HardwareMap hardwareMap) {
        leftMotorOne = hardwareMap.get(DcMotor.class, "left_drive_one");
        leftMotorOne.resetDeviceConfigurationForOpMode();

        leftMotorTwo = hardwareMap.get(DcMotor.class, "left_drive_two");
        leftMotorTwo.resetDeviceConfigurationForOpMode();

        rightMotorOne = hardwareMap.get(DcMotor.class, "right_drive_one");
        rightMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotorTwo = hardwareMap.get(DcMotor.class, "right_drive_two");
        rightMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = new IMU(hardwareMap);
    }

    public void drive(double forward, double turn) {
        drive(forward, 0.0d, turn);
    }

    public void drive(double forward, double strafe, double turn) {
        if(Math.abs(forward) <= Constants.DRIVE_INPUT_THRESHOLD) {
            forward = 0.0d;
        }
        if(Math.abs(strafe) <= Constants.DRIVE_INPUT_THRESHOLD) {
            strafe = 0.0d;
        }
        if(Math.abs(turn) <= Constants.DRIVE_INPUT_THRESHOLD) {
            turn = 0.0d;
        }

        if(turn != 0) {
            drive(Range.clip(
                    turn * getSpeed(), -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED
            ), Range.clip(
                    turn * getSpeed(), -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED
            ), Range.clip(
                    -turn * getSpeed(), -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED
            ), Range.clip(
                    -turn * getSpeed(), -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED
            ));
            return;
        }

        double targetAngle = Math.toDegrees(Math.atan2(forward, strafe));

        double leftAngleDiff = getLeftWheelAngle() - targetAngle;
        double rightWheelDiff = getRightWheelAngle() - targetAngle;

        if(
                Math.abs(leftAngleDiff) >= Constants.DRIVE_ANGLE_TOLERANCE ||
                Math.abs(leftAngleDiff) - 180 >= Constants.DRIVE_ANGLE_TOLERANCE ||
                Math.abs(leftAngleDiff) >= Constants.DRIVE_ANGLE_TOLERANCE ||
                Math.abs(leftAngleDiff) - 180 >= Constants.DRIVE_ANGLE_TOLERANCE
        ) {
            turnToAngle(targetAngle);
        }
    }

    public void drive(double p1, double p2, double p3, double p4) {
        leftMotorOne.setPower(p1);
        leftMotorTwo.setPower(p2);
        rightMotorOne.setPower(p3);
        rightMotorTwo.setPower(p4);
    }

    public void turnToAngle(double target) {
        double leftWheelDiff = getLeftWheelAngle() - target;
        double rightWheelDiff = getRightWheelAngle() - target;


    }

    public void resetWheels() {
        leftMotorOne.setPower(
                Range.clip(
                        -leftMotorOne.getCurrentPosition() / 100,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        leftMotorTwo.setPower(
                Range.clip(
                        -leftMotorTwo.getCurrentPosition() / 100,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        rightMotorOne.setPower(
                Range.clip(
                        -rightMotorOne.getCurrentPosition() / 100,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        rightMotorTwo.setPower(
                Range.clip(
                        -rightMotorTwo.getCurrentPosition() / 100,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );
    }

    public void stop() {
        leftMotorOne.setPower(0.0d);
        leftMotorTwo.setPower(0.0d);
        rightMotorOne.setPower(0.0d);
        rightMotorTwo.setPower(0.0d);
    }
}