package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.wrappers.SparkMINIMotor;

public class Claw {
    private Servo clawAngleServo;
    private Servo clawOpenServo;
    private SparkMINIMotor leftArmMotor;
    private SparkMINIMotor rightArmMotor;
    private Telemetry telemetry;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        clawAngleServo = hardwareMap.servo.get("claw_angle_servo");
        clawOpenServo = hardwareMap.servo.get("claw_open_servo");
        leftArmMotor = new SparkMINIMotor(hardwareMap, "arm_drive_left", "arm_drive_left_encoder");
        rightArmMotor = new SparkMINIMotor(hardwareMap, "arm_drive_right", "arm_drive_right_encoder");
    }

    public void raiseArm(){
         leftArmMotor.setTargetPosition(Constants.CLAW_OPENED_POSITION);
         leftArmMotor.setPower(Constants.CLAW_MOTOR_POWER);
         rightArmMotor.setTargetPosition(Constants.CLAW_OPENED_POSITION);
         rightArmMotor.setPower(Constants.CLAW_MOTOR_POWER);
    }

    public void lowerArm(){
         leftArmMotor.setTargetPosition(0);
         leftArmMotor.setPower(Constants.CLAW_MOTOR_POWER);
         rightArmMotor.setTargetPosition(0);
         rightArmMotor.setPower(Constants.CLAW_MOTOR_POWER);
    }

    public void update(){
        leftArmMotor.update();
        rightArmMotor.update();
    }

    public void stop() {

    }
}
