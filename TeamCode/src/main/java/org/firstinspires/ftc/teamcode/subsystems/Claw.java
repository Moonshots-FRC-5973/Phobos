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
    private Telemetry telemetry;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        clawAngleServo = hardwareMap.servo.get("claw_angle_servo");
        clawOpenServo = hardwareMap.servo.get("claw_open_servo");
    }
}
