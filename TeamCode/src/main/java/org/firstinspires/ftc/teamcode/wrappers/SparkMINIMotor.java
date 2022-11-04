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

    public SparkMINIMotor(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, name + "_encoder");
    }

    public SparkMINIMotor(HardwareMap hardwareMap, String name, String encoderName) {
        motor = hardwareMap.get(DcMotorSimple.class, name);
        encoder = hardwareMap.get(DigitalChannel.class, encoderName);
        prevState = encoder.getState();
    }

    public void update() {
        if(Math.abs(position - target) <= Constants.DRIVE_ANGLE_TOLERANCE) {
            target = position;
            power = 0.0d;
        }

        if(encoder.getState() != prevState) {
            if(power > 0) {
                position++;
            } else {
                position--;
            }
        }

        if(position > target) {
            motor.setPower(-power);
        } else {
            motor.setPower(power);
        }
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
        motor.close();
        encoder.close();
    }
}
