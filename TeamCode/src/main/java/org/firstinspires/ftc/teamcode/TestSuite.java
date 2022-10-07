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
                -gamepad1.left_stick_y,
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

        // Value (-360, 360)
        double wheelAngle = 360 * ((driveSystem.getLeftEncodersDifference() % Constants.ENCODER_COUNTS_PER_REV) / Constants.ENCODER_COUNTS_PER_REV);
        if(wheelAngle < 0)
            wheelAngle = 360 - wheelAngle; // [0, 360)

        double targetAngle = Math.atan2(forward, strafe); // (-180, 180]

        targetAngle += 180; // (0, 360]

        double angleDiff = targetAngle - wheelAngle;
        double rotationPower = (-Math.pow(angleDiff, 2) + (180 * angleDiff)) / Math.pow(90, 2);

        double m1Power, m2Power, m3Power, m4Power;

        if(targetAngle <= wheelAngle + 90 && targetAngle > wheelAngle) {
            // Q1
            telemetry.addData("Quadrant", "1");
        } else if(targetAngle <= wheelAngle + 180 && targetAngle >= wheelAngle + 90) {
            // Q2
            telemetry.addData("Quadrant", "2");
            staticPower *= -1;
            rotationPower *= -1;
        } else if(targetAngle < wheelAngle - 90 && targetAngle >= wheelAngle - 180) {
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

        m1Power = Range.clip(m1Power, -staticPower, staticPower);
        m2Power = Range.clip(m2Power, -staticPower, staticPower);
        m3Power = Range.clip(m3Power, -staticPower, staticPower);
        m4Power = Range.clip(m4Power, -staticPower, staticPower);

        telemetry.addData("Static Power", staticPower);
        telemetry.addData("Rotation Power", rotationPower);
        telemetry.addData("Wheel Angle", wheelAngle);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Motor Powers", String.format("%f, %f, %f, %f",
                m1Power, m2Power, m3Power, m4Power
        ));
        driveSystem.drive(m1Power, m2Power, m3Power, m4Power);
    }
}
