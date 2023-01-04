package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    private boolean gyroLocked = false;
    private double gyroTarget;
    private boolean turningToAngle = false;
    private double targetHeading;

    public MecanumDrive(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_motor_drive");
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_motor_drive");
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_motor_drive");
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_back_motor_drive");
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double forward, double strafe, double turn) {
        // If we're turning to an angle, continue turning to the angle and ignore drive inputs.
        if(turningToAngle) {
            turnRobotToAngle(targetHeading);
            return;
        }

        if(telemetry != null) {
            telemetry.addData("IMU", "Acceleration(%f, %f, %f)", imu.getXVelocity(), imu.getYVelocity(), imu.getZVelocity());
        }

        // DEADZONES
        if(Math.abs(forward) <= Constants.INPUT_THRESHOLD) {
            forward = 0.0d;
        }

        if(Math.abs(strafe) <= Constants.INPUT_THRESHOLD) {
            strafe = 0.0d;
        }

        if (isFieldCentric){
            // We want to adjust by IMU
            double target = Math.atan2(forward, strafe);
            double temp = forward;

            forward = forward * Math.cos(Math.toRadians(imu.getZAngle())) + strafe * Math.sin(Math.toRadians(imu.getZAngle()));
            strafe = -temp * Math.sin(Math.toRadians(imu.getZAngle())) + strafe * Math.cos(Math.toRadians(imu.getZAngle()));
        }

        // RIGHT STICK OVERRIDES ANY FORWARD/STRAFE
        if (Math.abs(turn) >= Constants.INPUT_THRESHOLD) {
            /*
            double xBoost = 0.0d; // Determine which Axes go to which direction. Only two are needed.
            double yBoost = 0.0d;
            double zBoost = 0.0d;
             */

            drive(turn, turn, -turn, turn);
            gyroLocked = false;
            return;
        }
        // If we're not turning, lock our gyro to track our intended heading
        if (! gyroLocked ) {
            gyroLocked = true;
            gyroTarget = imu.getZAngle();
        }

        // TUNING VARIABLES
        double gyroError = gyroTarget - imu.getZAngle();
        // if gyroError is positive, robot is rotating to the left, so left side should get more power
        double boost = (gyroError / Constants.DRIVE_ANGLE_TOLERANCE) / 180;
        if(telemetry != null) {
            telemetry.addData("Gyro Locked", gyroLocked);
            telemetry.addData("Gyro Target", gyroTarget);
            telemetry.addData("Error", gyroError);
        }
        drive(
                -forward - strafe - boost,
                -forward + strafe - boost,
                -forward + strafe + boost,
                forward + strafe - boost
        );
    }

    public void drive(double leftFront, double leftRear, double rightFront, double rightRear) {
        if(telemetry != null) {
            telemetry.addData("leftFront", leftFront);
            telemetry.addData("leftRear", leftRear);
            telemetry.addData("rightFront", rightFront);
            telemetry.addData("rightRear", rightRear);
        }

        leftFrontDrive.setPower(Range.clip(leftFront, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftBackDrive.setPower(Range.clip(leftRear, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightFrontDrive.setPower(Range.clip(rightFront, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightBackDrive.setPower(Range.clip(rightRear, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
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

    @Override
    public void turnRobotToAngle(double target) {
        gyroLocked = false;
        targetHeading = target;
        // Get the error. Positive means we need to rotate to the left
        double error = targetHeading - imu.getZAngle();
        // Ensure we don't move farther than one rotation
        error %= 360;
        telemetry.addData("Rot Error", error);
        // If we are within the requested tolerance (Constants.DRIVE_ANGLE_TOLERANCE), we should stop turning
        if(Math.abs(error) <= Constants.DRIVE_ANGLE_TOLERANCE) {
            stop();
            turningToAngle = false;
            return;
        } else {
            turningToAngle = true;
        }
        // As we approach the angle, we need to slow down the rotation
        double power = Range.clip(-error / 360, -0.5, 0.5);
        drive(power, power, -power, power);
    }
}