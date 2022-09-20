package org.firstinspires.ftc.teamcode.subsystems.driveSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.v1.constants.drivesystem.SwerveDriveConstants;

public class SwerveDrive implements DriveSystem {

    private DcMotor leftRotMotor;
    private DcMotor leftDriveMotor;
    private DcMotor rightRotMotor;
    private DcMotor rightDriveMotor;
    private boolean isRelativeAngle = false;

    public void makeRelativeAngleCalculations() {
        isRelativeAngle = true;
    }

    public void makeAbsoluteAngleCalculations() {
        isRelativeAngle = false;
    }

    public SwerveDrive(HardwareMap hardwareMap) {
        leftRotMotor = hardwareMap.get(DcMotor.class, SwerveDriveConstants.LEFT_ROT_MOTOR_NAME);
        leftDriveMotor = hardwareMap.get(DcMotor.class, SwerveDriveConstants.LEFT_DRIVE_MOTOR_NAME);

        rightRotMotor = hardwareMap.get(DcMotor.class, SwerveDriveConstants.RIGHT_ROT_MOTOR_NAME);
        rightDriveMotor = hardwareMap.get(DcMotor.class, SwerveDriveConstants.RIGHT_DRIVE_MOTOR_NAME);

        leftRotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightRotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double forward, double turn, double strafe) {
        // Condense Values
        forward = Range.clip(
                forward,
                -SwerveDriveConstants.MOTOR_MAXIMUM_POWER,
                SwerveDriveConstants.MOTOR_MAXIMUM_POWER
        );

        turn = Range.clip(
                turn,
                -SwerveDriveConstants.MOTOR_MAXIMUM_POWER,
                SwerveDriveConstants.MOTOR_MAXIMUM_POWER
        );

        strafe = Range.clip(
                strafe,
                -SwerveDriveConstants.MOTOR_MAXIMUM_POWER,
                SwerveDriveConstants.MOTOR_MAXIMUM_POWER
        );

        // Input Constraint Threshold
        if (Math.abs(forward) < SwerveDriveConstants.INPUT_THRESHOLD)
            forward = 0.0d;

        if (Math.abs(turn) < SwerveDriveConstants.INPUT_THRESHOLD)
            turn = 0.0d;

        if (Math.abs(strafe) < SwerveDriveConstants.INPUT_THRESHOLD)
            strafe = 0.0d;

        // Value Strafing over turning
        if(strafe != 0.0d && turn != 0.0d) {
            turn = 0.0d;
        }

        if(!isRelativeAngle) {
            //Relative to robot center
            if(turn == 0.0d) {
                double theta = Math.atan2(strafe, forward);
                leftRotMotor.setTargetPosition(0);
            } else {

            }
        } else {
            //Relative to driver direction
        }
    }

    public void drive(double forward, double turn) {
        drive(forward, turn, 0.0d);
    }

    public void turnToDegrees(double degrees) {

    }

    public void close() {
        leftRotMotor.setTargetPosition(0);
        leftRotMotor.setPower(SwerveDriveConstants.TURN_MOTOR_POWER_PERCENT);
        leftDriveMotor.setPower(0.0d);
        rightRotMotor.setTargetPosition(0);
        rightRotMotor.setPower(SwerveDriveConstants.TURN_MOTOR_POWER_PERCENT);
        rightDriveMotor.setPower(0.0d);
    }
}
