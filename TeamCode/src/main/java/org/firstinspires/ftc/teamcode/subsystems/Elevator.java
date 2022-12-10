package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.wrappers.SparkMINIMotor;

public class Elevator {
    private SparkMINIMotor leftArmControl;
    private SparkMINIMotor rightArmControl;
    private Telemetry telemetry;
    protected Claw claw;
    public Claw getClaw() {
        return claw;
    }

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.claw = new Claw(hardwareMap, telemetry);

        leftArmControl = new SparkMINIMotor(hardwareMap, "arm_drive_left", "arm_drive_encoder_left");
        rightArmControl = new SparkMINIMotor(hardwareMap, "arm_drive_right", "arm_drive_encoder_right");
    }
    // This stuff is just unfinished
    // Feel free to finish
    public void raiseElevator() {
        rightArmControl.setTargetPosition(-Constants.CLAW_OPENED_POSITION);
        leftArmControl.setTargetPosition(Constants.CLAW_OPENED_POSITION);
    }
    public void lowerElevator() {
        rightArmControl.setTargetPosition(Constants.CLAW_CLOSED_POSITION);
        leftArmControl.setTargetPosition(Constants.CLAW_CLOSED_POSITION);
    }

    // Need to call update() to ensure the Sparks hit the right values
    public void update() {
        double boost = (rightArmControl.getEncoderPosition() - leftArmControl.getEncoderPosition()) / 50;
        rightArmControl.setPower(Range.clip(Constants.CLAW_MOTOR_POWER - boost, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftArmControl.setPower(Range.clip(Constants.CLAW_MOTOR_POWER + boost, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftArmControl.update();
        rightArmControl.update();
        telemetry.addData("LeftArmMotor", leftArmControl.getEncoderPosition());
        telemetry.addData("RightArmMotor", rightArmControl.getEncoderPosition());
    }

    public void resetArm() {
        rightArmControl.setTargetPosition(Constants.CLAW_OPENED_POSITION);
        leftArmControl.setTargetPosition(-Constants.CLAW_OPENED_POSITION);

    }

    public void stop() {
            rightArmControl.setPower(0.0d);
            leftArmControl.setPower(0.0d);
    }
}
