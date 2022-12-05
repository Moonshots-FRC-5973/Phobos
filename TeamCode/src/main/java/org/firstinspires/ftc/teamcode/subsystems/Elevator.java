package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public Elevator (HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        this.claw = new Claw(hardwareMap, telemetry);

        leftArmControl = new SparkMINIMotor(hardwareMap, "left_arm_control", "left_arm_control_encoder");
        rightArmControl = new SparkMINIMotor(hardwareMap, "right_Arm_Control", "right_Arm_Control_Encoder");
    }
    //This stuff is just unfinished
    // Feel free to finish
    public void raiseElevator(){
        rightArmControl.setPower(0.5);
        rightArmControl.setTargetPosition(Constants.CLAW_OPENED_POSITION);
        leftArmControl.setPower(0.5);
        leftArmControl.setTargetPosition(Constants.CLAW_OPENED_POSITION);
    }
    public void lowerElevator() {
        rightArmControl.setPower(-0.5);
        rightArmControl.setTargetPosition(Constants.CLAW_OPENED_POSITION);
        leftArmControl.setPower(-0.5);
        leftArmControl.setTargetPosition(Constants.CLAW_OPENED_POSITION);
    }
    public void stop() {
            rightArmControl.setPower(0.0f);
            leftArmControl.setPower(0.0f);
    }
}
