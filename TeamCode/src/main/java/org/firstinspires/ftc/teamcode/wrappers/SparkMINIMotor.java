package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class SparkMINIMotor {
    private DcMotorSimple motor;
    private DigitalChannel encoder;
    private int position = 0;
    private boolean prevState;
    private int target;
    private double power;

    public SparkMINIMotor(HardwareMap hardwareMap, String name, String encoderName) {
        motor = hardwareMap.get(DcMotorSimple.class, name);
        encoder = hardwareMap.get(DigitalChannel.class, encoderName);
        prevState = encoder.getState();
    }

    public void update() {

        // hold the given position
        if(encoder.getState() != prevState) {
            if(position < target) {
                position++;
            } else {
                position--;
            }
        }

        if(Math.abs(position - target) <= 10) {
            target = position;
            power = 0.0d;
        }

        if(position > target) {
            motor.setPower(-Math.abs(power));
        } else {
            motor.setPower(Math.abs(power));
        }

        prevState = encoder.getState();
    }

    public void setTargetPosition(int position) {
        target = position;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public int getEncoderPosition() {
        return position;
    }

    public void close() {
        this.setPower(0);
        motor.close();
        encoder.close();
    }
}
