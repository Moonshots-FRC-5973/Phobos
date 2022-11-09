package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public class SwerveDrive {
    public final DcMotor leftMotorLeft;   // +Power = left wheel turning left
    public final DcMotor leftMotorRight;  // +Power = left wheel turning right
    public final DcMotor rightMotorLeft;  // +Power = right wheel turning right
    public final DcMotor rightMotorRight;
    private double speed = 1;
    private boolean fieldCentric = true;
    private final ElapsedTime timer;
    private final Telemetry telemetry;
    private final IMU imu;

    public int getLeftEncodersDifference() {
        return leftMotorLeft.getCurrentPosition() - leftMotorRight.getCurrentPosition();
    }
    public double getLeftWheelAngle() {
        double angle = (360 * (getLeftEncodersDifference() % Constants.DRIVE_ENCODER_COUNTS_PER_REV) / Constants.DRIVE_ENCODER_COUNTS_PER_REV);
        if(fieldCentric) return angle + imu.getZAngle();
        return angle;
    }
    public int getRightEncodersDifference() {
        return rightMotorLeft.getCurrentPosition() - rightMotorRight.getCurrentPosition();
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

    public SwerveDrive(HardwareMap hardwareMap, ElapsedTime timer, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftMotorLeft = hardwareMap.get(DcMotor.class, "left_drive_left");
        leftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotorRight = hardwareMap.get(DcMotor.class, "left_drive_right");
        leftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorLeft = hardwareMap.get(DcMotor.class, "right_drive_left");
        rightMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorRight = hardwareMap.get(DcMotor.class, "right_drive_right");
        rightMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = new IMU(hardwareMap);

        this.timer = timer;
    }

    public void drive(double forward, double strafe, double turn) {
        if(Math.abs(forward) <= Constants.DRIVE_INPUT_THRESHOLD) {
            forward = 0.0d;
        }
        if(Math.abs(strafe) <= Constants.DRIVE_INPUT_THRESHOLD) {
            strafe = 0.0d;
        }
        if(Math.abs(turn) >= Constants.DRIVE_INPUT_THRESHOLD) {
            drive(
                    (-turn * getSpeed()),
                    (-turn * getSpeed()),
                    (turn * getSpeed()),
                    (turn * getSpeed())
            );
            return;
        }

        // Translate XY input to degree and power
        double targetAngle = Math.atan2(forward, strafe); // -180 - 180
        double power = Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2));
        double leftAngleDiff = getLeftWheelAngle() - targetAngle;

        double ltPower = getLeftWheelAngle();

        // Quadrant Check to move in the correct direction / fastest direction
        if(leftAngleDiff <= 0 && leftAngleDiff > 90) {
            // Quadrant: 1
        } else if(leftAngleDiff <= 90 && leftAngleDiff > 180) {
            // Quadrant: 2
        } else if(leftAngleDiff <= 180 && leftAngleDiff > 270) {
            // Quadrant: 3
        } else {
            // Quadrant: 4
        }

        // Wheel |  Motor
        // <l, r><l, r>Power
        double llPower = power;
        double lrPower = power;
        double rlPower = power;
        double rrPower = power;

        if(telemetry != null) {// In case no messages should be sent.
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Power", power);
            telemetry.addData("Left Wheel Angle", getLeftWheelAngle());
            telemetry.addData("Right Wheel Angle", getRightWheelAngle());
            telemetry.addData("Left Wheel Power", "%f%f", llPower, lrPower);
        }

        drive(llPower, lrPower, rlPower, rrPower);
    }

    public void drive(double p1, double p2, double p3, double p4) {
        leftMotorLeft.setPower(Range.clip(p1, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftMotorRight.setPower(Range.clip(p2, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightMotorLeft.setPower(Range.clip(p3, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightMotorRight.setPower(Range.clip(p4, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
    }

    public void turnWheelToAngle(double target) {
        if(Math.abs(getLeftWheelAngle() - target) >= Constants.DRIVE_ANGLE_TOLERANCE) {
            leftMotorLeft.setPower(-0.75d);
            leftMotorRight.setPower(0.75d);
        }
        if(Math.abs(getRightWheelAngle() - target) >= Constants.DRIVE_ANGLE_TOLERANCE) {
            rightMotorLeft.setPower(-0.75d);
            rightMotorRight.setPower(0.75d);
        }
    }
    public void resetWheels() {
        leftMotorLeft.setPower(
                Range.clip(
                        -leftMotorLeft.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        leftMotorRight.setPower(
                Range.clip(
                        -leftMotorRight.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        rightMotorLeft.setPower(
                Range.clip(
                        -rightMotorLeft.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        rightMotorRight.setPower(
                Range.clip(
                        -rightMotorRight.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );
    }
    public void stop() {
        leftMotorLeft.setPower(0.0d);
        leftMotorRight.setPower(0.0d);
        rightMotorLeft.setPower(0.0d);
        rightMotorRight.setPower(0.0d);
    }
}