package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.SparkMINIMotor;

public class Elevator {
    private SparkMINIMotor leftArmControl;
    private SparkMINIMotor rightArmControl;
    private Telemetry telemetry;
    public Elevator (HardwareMap hardwareMap, Telemetry telemetry){
        leftArmControl = new SparkMINIMotor(hardwareMap, "left_arm_control", "left_arm_control_encoder");
        rightArmControl = new SparkMINIMotor(hardwareMap, "right_Arm_Control", "right_Arm_Control_Encoder");
    }

}
