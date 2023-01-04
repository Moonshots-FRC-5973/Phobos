package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Claw {
    private int encoderOffset;
    public void setEncoderOffset() {
        encoderOffset = leftArmMotor.getCurrentPosition();
    }
    private DcMotor leftArmMotor;
    private DcMotor rightArmMotor;
    private Servo leftClawHeightServo;
    private Servo clawOpenServo;

    private Telemetry telemetry;

    private double armServoPosition = 0;
    public int getCurrentHeight() {
        return leftArmMotor.getCurrentPosition();
    }
    private double clawServoPosition = 0;
    private int targetPosition = 0;

    public enum Position {
        MIN,
        LOW,
        MID,
        HIGH
    }

    public enum Status {
        OPEN,
        CLOSE
    }

    private Position position;
    public Position getPosition() {
        return position;
    }

    private Status status;
    public Status getStatus() {
        return status;
    }

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        clawOpenServo = hardwareMap.servo.get("claw_open_servo");
        leftArmMotor = hardwareMap.get(DcMotor.class, "left_motor_arm");
        rightArmMotor = hardwareMap.get(DcMotor.class, "right_motor_arm");
        leftClawHeightServo = hardwareMap.get(Servo.class, "left_claw_height_servo");

        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);
        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
        leftClawHeightServo.setPosition(0);
        clawOpenServo.setPosition(Constants.CLAW_CLOSED_POSITION);


    }

    public void start() {
        clawServoPosition = Constants.CLAW_CLOSED_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_MIN_POSITION;
        position = Position.MIN;
        run();
    }

    public void setMin(){
        targetPosition = Constants.ARM_DOWN_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_MIN_POSITION;
        position = Position.MIN;
        run();
    }

    public void setLow(){
        targetPosition = Constants.ARM_LOW_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_LOW_POSITION;
        position = Position.LOW;
        run();

    }

    public void setMid(){
        targetPosition = Constants.ARM_MID_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_MID_POSITION;
        position = Position.MID;
        run();
    }

    public void setHigh(){
        targetPosition = Constants.ARM_HIGH_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_MAX_POSITION;
        position = Position.HIGH;
        run();
    }

    public void open() {
        clawServoPosition = Constants.CLAW_OPENED_POSITION;
        status = Status.OPEN;
        run();
    }

    public void close() {
        clawServoPosition = Constants.CLAW_CLOSED_POSITION;
        status = Status.CLOSE;
        run();
    }

    private void run(){
        leftArmMotor.setPower(Constants.ARM_MOTOR_POWER);
        leftArmMotor.setTargetPosition(targetPosition + encoderOffset);
        leftArmMotor.setMode(RUN_TO_POSITION);
        rightArmMotor.setPower(Constants.ARM_MOTOR_POWER);
        rightArmMotor.setTargetPosition(targetPosition + encoderOffset);
        rightArmMotor.setMode(RUN_TO_POSITION);
        telemetry.addData("LAM", leftArmMotor.getCurrentPosition());
        telemetry.addData("RAM", rightArmMotor.getCurrentPosition());
        telemetry.addData("Servo Target", armServoPosition);
        leftClawHeightServo.setPosition(armServoPosition);
        clawOpenServo.setPosition(clawServoPosition);
        telemetry.addData("Open", clawOpenServo.getPosition());
        telemetry.addData("Open target", clawServoPosition);
    }

    public void lowerClawBySpeed(double speed) {
        leftArmMotor.setMode(RUN_USING_ENCODER);
        rightArmMotor.setMode(RUN_USING_ENCODER);
        leftArmMotor.setPower(speed);
        rightArmMotor.setPower(speed);
    }

    public void stop(){
        leftArmMotor.setPower(0.0d);
        rightArmMotor.setPower(0.0d);
    }


}
