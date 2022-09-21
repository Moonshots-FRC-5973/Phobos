package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Claw {
    private Servo armServo;
    double  MIN_POSITION = 0, MAX_POSITION = 1;
    double armPosition = .5;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;


    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        armServo = hardwareMap.servo.get(Constants.ARM_SERVO_NAME);
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
