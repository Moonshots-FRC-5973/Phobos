package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.drives.SwerveDrive;

//Test Suite Code. Testing Swerve drive
@TeleOp(name="Test Suite", group="TeleOp")
public class TestSuite extends OpMode {
    private SwerveDrive driveSystem;
    private boolean movingToAngle = false;
    private double targetAngle = 0;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        driveSystem = new SwerveDrive(hardwareMap, new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS));
        driveSystem.makeRobotCentric();
        telemetry.addData("Status", "Initialized.");
        targetAngle = 180;
        movingToAngle = true;
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {


        if(gamepad1.a) {
            movingToAngle = true;
            targetAngle = 180;
        }

        if(gamepad1.b) {
            movingToAngle = true;
            targetAngle = 90;
        }

        if(gamepad1.y) {
            movingToAngle = true;
            targetAngle = 0;
        }

        if(gamepad1.x) {
            movingToAngle = true;
            targetAngle = 270;
        }

        if(movingToAngle) {
            telemetry.addData("Mode", "TTA");
            turnToAngle(targetAngle);
        } else {
            telemetry.addData("Mode", "DR1");
            drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        }

        telemetry.addData("Left Angle Difference", driveSystem.getLeftEncodersDifference());
        telemetry.update();
    }

    @Override
    public void stop() {
        driveSystem.stop();
    }

    private void drive(double forward, double strafe, double turn) {
        telemetry.addData("Inputs", String.format("%f, %f, %f", forward, strafe, turn));

        if(Math.abs(forward) <= Constants.DRIVE_INPUT_THRESHOLD) {
            forward = 0.0d;
        }

        if(Math.abs(strafe) <= Constants.DRIVE_INPUT_THRESHOLD) {
            strafe = 0.0d;
        }

        if(Math.abs(turn) <= Constants.DRIVE_INPUT_THRESHOLD) {
            turn = 0.0d;
        }

        double targetAngle = Math.toDegrees(Math.atan2(strafe, forward));
        telemetry.addData("Target Angle", targetAngle);

        double leftWheelAngle = driveSystem.getLeftWheelAngle();
        telemetry.addData("Left Wheel Angle", leftWheelAngle);
        double rightWheelAngle = driveSystem.getRightWheelAngle();
        telemetry.addData("Right Wheel Angle", rightWheelAngle);
        double leftAngleDiff = targetAngle - leftWheelAngle;
        telemetry.addData("Left Angle Diff", leftAngleDiff);
        double rightAngleDiff = targetAngle - leftWheelAngle;
        telemetry.addData("Right Angle Diff", rightAngleDiff);

        forward = Range.clip(forward, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED);
        turn = Range.clip(turn, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED);

        if(turn != 0) {
            driveSystem.drive(turn, turn, -turn, -turn);
        } else if(Math.abs(leftAngleDiff) >= Constants.DRIVE_ANGLE_TOLERANCE) {
            turnToAngle(targetAngle);
        } else {
            driveSystem.drive(forward, forward, forward, forward);
        }
    }

    private void turnToAngle(double target) {
        telemetry.addData("Target Angle", target);
        double leftWheelAngle = driveSystem.getLeftWheelAngle();
        telemetry.addData("Left Wheel Angle", leftWheelAngle);
        double rightWheelAngle = driveSystem.getRightWheelAngle();
        telemetry.addData("Right Wheel Angle", rightWheelAngle);


        double leftAngleDiff = target - leftWheelAngle;
        telemetry.addData("Left Angle Diff", leftAngleDiff);
        double leftRotationPower = Range.clip(
                Constants.DRIVE_ROTATION_POWER_MODIFIER * Math.sin(Math.toRadians(leftAngleDiff)),
                -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED);
        double rightAngleDiff = target - rightWheelAngle;
        telemetry.addData("Right Angle Diff", rightAngleDiff);
        double rightRotationPower = Range.clip(
                Constants.DRIVE_ROTATION_POWER_MODIFIER * Math.sin(Math.toRadians(rightAngleDiff)),
                -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED);

        if(
                (Math.abs(leftAngleDiff) <= Constants.DRIVE_ANGLE_TOLERANCE ||
                        Math.abs(leftAngleDiff) - 180 <= Constants.DRIVE_ANGLE_TOLERANCE ) &&
                (Math.abs(rightAngleDiff) <= Constants.DRIVE_ANGLE_TOLERANCE ||
                        Math.abs(rightAngleDiff) - 180 <= Constants.DRIVE_ANGLE_TOLERANCE)
        ) {
            driveSystem.stop();
            movingToAngle = false;
        } else {
            driveSystem.drive(leftRotationPower, -leftRotationPower,
                    rightRotationPower, -rightRotationPower);
        }
    }
}
