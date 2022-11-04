package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.wrappers.SparkMINIMotor;

public class Claw {
    private Servo armServo;
    private SparkMINIMotor leftArmMotor;
    private SparkMINIMotor rightArmMotor;
    private static final double  MIN_POSITION = 0;
    private static final double MAX_POSITION = 1;
    private double armPosition;
    private Telemetry telemetry;


    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        armServo = hardwareMap.servo.get(Constants.ARM_SERVO_NAME);
        leftArmMotor = new SparkMINIMotor(hardwareMap, "arm_drive_left", "arm_drive_left_encoder");
        rightArmMotor = new SparkMINIMotor(hardwareMap, "arm_drive_right", "arm_drive_right_encoder");
        armPosition = 0.5;
    }

    public void open(){
         if(armPosition > MIN_POSITION) {
             armPosition -= .01;
             execute();
         }
    }

    public void close(){
         if(armPosition < MAX_POSITION) {
             armPosition += .01;
             execute();
         }
    }

    private void execute(){
        armServo.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
        telemetry.addData("arm servo", "position=" + armPosition + "  actual=" + armServo.getPosition());
    }
}
