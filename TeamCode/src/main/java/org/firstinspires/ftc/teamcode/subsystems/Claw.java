package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * operates the claw + elevator
 */
public class Claw {
    /**
     * Describes the current height of the arm
     */
    public enum Position {
        MIN,
        LOW,
        MID,
        HIGH,
        PICKUP,
        CUSTOM
    }

    /**
     * Describes the current status of the claw
     */
    public enum Status {
        OPEN,
        CLOSE
    }

    private int leftEncoderOffset;
    private int rightEncoderOffset;
    private DcMotor leftArmMotor;
    private DcMotor rightArmMotor;
    private Servo leftClawHeightServo;
    private Servo clawOpenServo;
    private Telemetry telemetry;
    private double armServoPosition = 0;
    private double clawServoPosition = 0;
    private int targetPosition = 0;
    private Position position;
    private Status status;

    /**
     * Activates the movement of the arm
     */
    private void run() {
        if(position == Position.MIN) {
            leftArmMotor.setMode(RUN_WITHOUT_ENCODER);
            rightArmMotor.setMode(RUN_WITHOUT_ENCODER);
        }
        //Adjust power based on arm target position
        /*
        switch (position) {
            case MIN:
                leftArmMotor.setPower(0.0d);
                rightArmMotor.setPower(0.0d);
                break;
            case MID:
                leftArmMotor.setPower(Constants.ARM_MOTOR_POWER - 0.15);
                rightArmMotor.setPower(Constants.ARM_MOTOR_POWER - 0.15);
                break;
            case HIGH:
                leftArmMotor.setPower(Constants.ARM_MOTOR_POWER - 0.25);
                rightArmMotor.setPower(Constants.ARM_MOTOR_POWER - 0.25);
                break;
            default:
                leftArmMotor.setPower(Constants.ARM_MOTOR_POWER);
                rightArmMotor.setPower(Constants.ARM_MOTOR_POWER);
        }

         */
        leftArmMotor.setPower(Constants.ARM_MOTOR_POWER);
        rightArmMotor.setPower(Constants.ARM_MOTOR_POWER);
        if(position != Position.MIN) {
            leftArmMotor.setTargetPosition(targetPosition + leftEncoderOffset);
            leftArmMotor.setMode(RUN_TO_POSITION);
            rightArmMotor.setTargetPosition(targetPosition + rightEncoderOffset);
            rightArmMotor.setMode(RUN_TO_POSITION);
        }

        telemetry.addData("LAM", leftArmMotor.getCurrentPosition());
        telemetry.addData("RAM", rightArmMotor.getCurrentPosition());
        telemetry.addData("Servo Target", armServoPosition);

        if(leftClawHeightServo.getPosition() >= armServoPosition || position == Position.CUSTOM)
            leftClawHeightServo.setPosition(armServoPosition);
        clawOpenServo.setPosition(clawServoPosition);

        telemetry.addData("Open", clawOpenServo.getPosition());
        telemetry.addData("Open target", clawServoPosition);
    }

    /**
     * sets the current ground level of the arm encoders
     */
    public void setEncoderOffset() {
        leftEncoderOffset = leftArmMotor.getCurrentPosition();
        rightEncoderOffset = rightArmMotor.getCurrentPosition();
    }

    /**
     * @return Returns the left arm motor's current encoder value
     */
    public int getLeftCurrentHeight() {
        return leftArmMotor.getCurrentPosition();
    }

    /**
     * @return Returns the right arm motor's current encoder value
     */
    public int getRightCurrentHeight() {
        return rightArmMotor.getCurrentPosition();
    }

    /**
     * @return an enum specifying the height of the arm lift
     */
    public Position getPosition() {
        return position;
    }

    /**
     * @return an enum specifying if the claw is open or closed
     */
    public Status getStatus() {
        return status;
    }

    /**
     * Generates a new claw subsystem.
     * @param hardwareMap the hardware map of the robot
     * @param telemetry the output stream (as an instance of Telemetry) to use for debugging
     */
    public Claw(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        clawOpenServo = hardwareMap.servo.get("claw_open_servo");
        leftArmMotor = hardwareMap.get(DcMotor.class, "left_motor_arm");
        rightArmMotor = hardwareMap.get(DcMotor.class, "right_motor_arm");
        leftClawHeightServo = hardwareMap.get(Servo.class, "left_claw_height_servo");

        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);
        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
        leftClawHeightServo.setPosition(0);
        clawOpenServo.setPosition(Constants.CLAW_CLOSED_POSITION);
        position = Position.MIN;
        setEncoderOffset();
    }

    /**
     * Moves the claw to the ground to pick up cones.
     */
    public void setMin(){
        targetPosition = Constants.ARM_DOWN_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_MIN_POSITION;
        position = Position.MIN;
        run();
    }

    public void setGroundPickup() {
        targetPosition = Constants.ARM_PICKUP_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_PICKUP_POSITION;
        position = Position.PICKUP;
        run();
    }

    /**
     * Moves the claw to the height for the low pole
     */
    public void setLow(){
        targetPosition = Constants.ARM_LOW_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_LOW_POSITION;
        position = Position.LOW;
        run();
    }

    /**
     * Moves the claw to the height for the mid height pole
     */
    public void setMid(){
        targetPosition = Constants.ARM_MID_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_MID_POSITION;
        position = Position.MID;
        run();
    }

    /**
     * Moves the claw to the height for the highest pole
     */
    public void setHigh(){
        targetPosition = Constants.ARM_HIGH_POSITION;
        armServoPosition = Constants.CLAW_HEIGHT_MAX_POSITION;
        position = Position.HIGH;
        run();
    }

    /**
     * Opens the claw
     */
    public void open() {
        clawServoPosition = Constants.CLAW_OPENED_POSITION;
        status = Status.OPEN;
        run();
    }

    /**
     * Closes the claw
     */
    public void close() {
        clawServoPosition = Constants.CLAW_CLOSED_POSITION;
        status = Status.CLOSE;
        run();
    }

    /**
     * Angles the claw
     */
    public void angleClaw() {
        leftClawHeightServo.setPosition(armServoPosition);
    }

    /**
     * @param speed Adjusts the claw based on speed instead of by encoder
     */
    public void lowerClawBySpeed(double speed) {
        targetPosition -= speed * 20;
        position = Position.CUSTOM;
        run();
    }

    public void adjustClawAngle(double change) {
        //armServoPosition = Range.clip((change / 40) + armServoPosition, -1, 1);
        armServoPosition += (change / 137);
        position = Position.CUSTOM;
        run();
    }

    /**
     * Stops the motors
     */
    public void stop() {
        leftClawHeightServo.setPosition(0.0d);
        leftArmMotor.setMode(RUN_WITHOUT_ENCODER);
        leftArmMotor.setPower(0.0d);
        rightArmMotor.setMode(RUN_WITHOUT_ENCODER);
        rightArmMotor.setPower(0.0d);
    }


}
