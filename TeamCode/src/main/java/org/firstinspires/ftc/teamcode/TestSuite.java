package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.degrees;
import org.firstinspires.ftc.teamcode.subsystems.drives.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.sli.Vec2;

@TeleOp(name="Test Suite", group="TeleOp")
public class TestSuite extends OpMode {
    private SwerveDrive driveSystem;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        driveSystem = new SwerveDrive(hardwareMap);
        driveSystem.makeRobotCentric();
        telemetry.addData("Status", "Initialized.");
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        drive(
                new Vec2(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y
                ),
                gamepad1.right_stick_x
        );

        if(gamepad1.a) {
            driveSystem.switchMode();
        }

        if(gamepad1.b) {
            driveSystem.resetWheels();
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        driveSystem.stop();
    }

    public void drive(Vec2 movement, double turn) {
        if(Math.abs(movement.getX()) <= Constants.INPUT_THRESHOLD) {
            movement.setX(0.0d);
        }

        if(Math.abs(movement.getY()) <= Constants.INPUT_THRESHOLD) {
            movement.setY(0.0d);
        }

        if(Math.abs(turn) <= Constants.INPUT_THRESHOLD) {
            turn = 0.0d;
        }

        telemetry.addData("Left Motor 1 Encoder Value", driveSystem.leftMotorOne.getCurrentPosition());
        telemetry.addData("Left Motor 2 Encoder Value", driveSystem.leftMotorTwo.getCurrentPosition());

        double encoderDiff =
                driveSystem.leftMotorOne.getCurrentPosition() -
                        driveSystem.leftMotorTwo.getCurrentPosition();
        telemetry.addData("Encoder Difference", encoderDiff);
        double wheelAngle = (encoderDiff / Constants.ENCODER_COUNTS_PER_REV) * 360;
        telemetry.addData("Wheel Angle", wheelAngle);

        if(driveSystem.isFieldCentric()) {
            telemetry.addData("DriveMode","Field Centric");
            double angle = Math.atan2(movement.getX(), movement.getY());
            angle *= 180 / Math.PI;
            double power = Math.sqrt(Math.pow(movement.getX(), 2) + Math.pow(movement.getY(), 2)) ;
            telemetry.addData("Args", "Power %f, Encoder Target %f", power, angle);
            double angleDiff;
            if(angle >= 0) {
                angleDiff = angle - wheelAngle;
            } else {
                angleDiff = wheelAngle + angle;
            }
            /*
            telemetry.addData("Left Motor 1 Power",
                    power * ((driveSystem.leftMotorOne.getCurrentPosition() % Constants.ENCODER_COUNTS_PER_REV) - angle) / 10);
            telemetry.addData("Left Motor 2 Power",
                    power * (angle - (driveSystem.leftMotorTwo.getCurrentPosition() % Constants.ENCODER_COUNTS_PER_REV)) / 10);

            driveSystem.drive(
                    Range.clip(
                            power * (driveSystem.leftMotorOne.getCurrentPosition() - angle),
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED
                    ),
                    Range.clip(
                            power * (angle - driveSystem.leftMotorTwo.getCurrentPosition()),
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED
                    ),
                    0.0d,
                    0.0d
            );

             */



        } else {
            driveSystem.drive(
                    Range.clip(
                            movement.getY() + turn,
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED
                    ),
                    Range.clip(
                            movement.getY() - turn,
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED
                    ),
                    Range.clip(
                            movement.getY() + turn,
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED),
                    Range.clip(
                            movement.getY() - turn,
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED
                    )
            );
        }
    }
}
