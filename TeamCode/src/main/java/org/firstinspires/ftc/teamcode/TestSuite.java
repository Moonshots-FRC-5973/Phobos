package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.degrees;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.subsystems.drives.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.sli.Vec2;
//Test Suite Code. Testing Swerve drive
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
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
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

    private void drive(double forward, double strafe, double turn) {
        telemetry.addData("Inputs", String.format("%f, %f, %f", forward, strafe, turn));
        double staticPower = Range.clip(
                Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2) + Math.pow(turn, 2)),
                -Constants.MOTOR_MAX_SPEED,
                Constants.MOTOR_MAX_SPEED
        );
        telemetry.addData("Static Power", staticPower);

        double m1Power, m2Power, m3Power, m4Power;

        double leftWheelAngle = (360 * (driveSystem.getLeftEncodersDifference() / Constants.ENCODER_COUNTS_PER_REV)) % 360;
        telemetry.addData("Left Wheel Angle", leftWheelAngle);
        double rightWheelAngle = (360 * (driveSystem.getRightEncodersDifference() / Constants.ENCODER_COUNTS_PER_REV)) % 360;
        telemetry.addData("Right Wheel Angle", rightWheelAngle);

        double targetAngle = Math.toDegrees(Math.atan2(strafe, forward));
        /*

        double targetAngle = Math.toDegrees(Math.atan2(strafe, forward)); // (-180, 180]

        targetAngle += 180; // (0, 360]

        double angleDiff = targetAngle - wheelAngle;
        double rotationPower = Math.abs(Math.sin(Math.toRadians(angleDiff)));


        if(targetAngle <= wheelAngle + 90 && targetAngle > wheelAngle) {
            // Q1
            telemetry.addData("Quadrant", "1");
        } else if(targetAngle <= wheelAngle + 180 && targetAngle >= wheelAngle + 90) {
            // Q2
            telemetry.addData("Quadrant", "2");
            staticPower *= -1;
            rotationPower *= -1;
        } else if(targetAngle < wheelAngle + 270 && targetAngle >= wheelAngle + 180) {
            // Q3
            telemetry.addData("Quadrant", "3");
            staticPower *= -1;
        } else {
            // Q4
            telemetry.addData("Quadrant", "4");
            rotationPower *= -1;
        }

        m1Power = staticPower + rotationPower + turn;
        m2Power = staticPower - rotationPower + turn;
        m3Power = staticPower + rotationPower - turn;
        m4Power = staticPower - rotationPower - turn;

        m1Power = scale(m1Power, -3, 3, -staticPower, staticPower);
        m2Power = scale(m2Power, -3, 3, -staticPower, staticPower);
        m3Power = scale(m3Power, -3, 3, -staticPower, staticPower);
        m4Power = scale(m4Power, -3, 3, -staticPower, staticPower);

        telemetry.addData("Static Power", staticPower);
        telemetry.addData("Rotation Power", rotationPower);
        telemetry.addData("Wheel Angle", wheelAngle);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Motor Powers", String.format("%f, %f, %f, %f",
                m1Power, m2Power, m3Power, m4Power
        ));

         */
        //driveSystem.drive(m1Power, m2Power, m3Power, m4Power);
    }

    private double scale(double num, double smin, double smax, double fmin, double fmax) {
        return ((fmax - fmin) * ((num - smin) / (smax - smin))) + fmin;
    }
}
