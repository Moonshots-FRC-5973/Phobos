package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public class SwerveDrive {
    private final DcMotor leftMotorOne;
    private final DcMotor leftMotorTwo;
    private final DcMotor rightMotorOne;
    private final DcMotor rightMotorTwo;
    private double speed = 1;
    private boolean fieldCentric = true;
    private final ElapsedTime timer;

    private final IMU imu;

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
        double angle = (360 * (getRightEncodersDifference() % Constants.DRIVE_ENCODER_COUNTS_PER_REV) / Constants.DRIVE_ENCODER_COUNTS_PER_REV);
        if (fieldCentric) return angle + imu.getZAngle();
        return angle;
    }
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
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
    public void upSpeed(double amount) {
        speed += amount;
    }
    public void dropSpeed(double amount) {
        speed -= amount;
    }

    public SwerveDrive(HardwareMap hardwareMap, ElapsedTime timer) {
        leftMotorOne = hardwareMap.get(DcMotor.class, "left_drive_one");
        leftMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotorTwo = hardwareMap.get(DcMotor.class, "left_drive_two");
        leftMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorOne = hardwareMap.get(DcMotor.class, "right_drive_one");
        rightMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorTwo = hardwareMap.get(DcMotor.class, "right_drive_two");
        rightMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = new IMU(hardwareMap);
        this.timer = timer;
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
            double leftPower = Math.abs(getLeftWheelAngle()) *
                    Math.abs(Math.sin(Math.toRadians(getLeftWheelAngle())));
            double rightPower = Math.abs(getRightWheelAngle()) *
                    Math.abs(Math.sin(Math.toRadians(getRightWheelAngle())));

            drive(
                    (turn * getSpeed()) + leftPower,
                    (turn * getSpeed()) - leftPower,
                    (-turn * getSpeed()) + rightPower,
                    (-turn * getSpeed()) - rightPower
            );
            return;
        }

        double staticPower = Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2));
        double targetAngle = Math.atan2(forward, strafe);

        if(isFieldCentric()) {
            targetAngle -= imu.getZAngle();
        }

        double leftWheelDiff = targetAngle - getLeftWheelAngle();
        double rightWheelDiff = targetAngle - getRightWheelAngle();

        double leftPower = Math.abs(leftWheelDiff) *
                Math.abs(Math.sin(Math.toRadians(leftWheelDiff)));
        double rightPower = Math.abs(rightWheelDiff) *
                Math.abs(Math.sin(Math.toRadians(rightWheelDiff)));

        drive(
                staticPower + leftPower,
                staticPower - leftPower,
                staticPower + rightPower,
                staticPower - rightPower
        );



    }

    public void drive(double p1, double p2, double p3, double p4) {
        leftMotorOne.setPower(Range.clip(p1, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftMotorTwo.setPower(Range.clip(p2, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightMotorOne.setPower(Range.clip(p3, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightMotorTwo.setPower(Range.clip(p4, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
    }

    public void turnWheelToAngle(double target) {
        double leftWheelDiff = getLeftWheelAngle() - target;
        double rightWheelDiff = getRightWheelAngle() - target;

        double leftRot = Math.abs(Constants.DRIVE_ROTATION_POWER_MODIFIER * Math.sin(Math.toRadians(leftWheelDiff)));
        double rightRot = Math.abs(Constants.DRIVE_ROTATION_POWER_MODIFIER * Math.sin(Math.toRadians(rightWheelDiff)));

        drive(leftRot, -leftRot, rightRot, -rightRot);
    }
    public void resetWheels() {
        leftMotorOne.setPower(
                Range.clip(
                        -leftMotorOne.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        leftMotorTwo.setPower(
                Range.clip(
                        -leftMotorTwo.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        rightMotorOne.setPower(
                Range.clip(
                        -rightMotorOne.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        rightMotorTwo.setPower(
                Range.clip(
                        -rightMotorTwo.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
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