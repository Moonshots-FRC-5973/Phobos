package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.sensors.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.sli.Vec2;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public class SwerveDrive {
    public DcMotor leftMotorOne;
    public DcMotor leftMotorTwo;
    private DcMotor rightMotorOne;
    private DcMotor rightMotorTwo;
    private boolean fieldCentric = true;

    private IMU imu;

    public int getLeftEncodersDifference() {
        return leftMotorOne.getCurrentPosition() - leftMotorTwo.getCurrentPosition();
    }

    public void makeFieldCentric() {
        fieldCentric = true;
    }

    public void makeRobotCentric() {
        fieldCentric = false;
    }

    public void switchMode() {
        fieldCentric = !fieldCentric;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public SwerveDrive(HardwareMap hardwareMap) {
        leftMotorOne = hardwareMap.get(DcMotor.class, "left_drive_one");
        leftMotorOne.resetDeviceConfigurationForOpMode();
        leftMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotorTwo = hardwareMap.get(DcMotor.class, "left_drive_two");
        leftMotorTwo.resetDeviceConfigurationForOpMode();
        leftMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        rightMotorOne = hardwareMap.get(DcMotor.class, "right_drive_one");
        rightMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorTwo = hardwareMap.get(DcMotor.class, "right_drive_two");
        rightMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        imu = new IMU(hardwareMap);
    }

    public void init() {

    }

    public void drive(double forward, double turn) {
        drive(forward, 0.0d, turn);
    }

    public void drive(Vec2 movement, double turn) {
        if(Math.abs(movement.getX()) <= Constants.INPUT_THRESHOLD) {
            movement.setX(0.0d);
        }

        if(Math.abs(movement.getY()) <= Constants.INPUT_THRESHOLD) {
            movement.setY(0.0d);
        }

        if(Math.abs(turn) <= Constants.INPUT_THRESHOLD) {
            turn = 0.0d;
        }

        if(fieldCentric) {
            int encoderDiff = leftMotorOne.getCurrentPosition() - leftMotorTwo.getCurrentPosition();
            double wheelAngle = (encoderDiff / Constants.ENCODER_COUNTS_PER_REV) * 360;
            double power = Math.sqrt(Math.pow(movement.getX(), 2) + Math.pow(movement.getY(), 2));
            double angle = (Math.atan2(movement.getX(), movement.getY()) * 180) / Math.PI;
            double m1Power = ((angle - wheelAngle) / 10);
            double m2Power = - ((angle - wheelAngle) / 10);
            m1Power *= power;
            m2Power *= power;

            drive(Range.clip(
                    m1Power, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED
            ), Range.clip(
                    m2Power, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED
            ), 0.0d, 0.0d);
        } else {
            drive(
                    Range.clip(
                            movement.getY() + turn,
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED
                    ),
                    Range.clip(
                            movement.getY() - turn,
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED
                    ),
                    Range.clip(
                            movement.getY() + turn,
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED),
                    Range.clip(
                            movement.getY() - turn,
                            -Constants.MOTOR_MAX_SPEED,
                            Constants.MOTOR_MAX_SPEED
                    )
            );
        }
    }

    public void drive(double forward, double strafe, double turn) {
        drive(new Vec2(forward, strafe), turn);
    }

    public void drive(double p1, double p2, double p3, double p4) {
        leftMotorOne.setPower(p1);
        leftMotorTwo.setPower(p2);
        //rightMotorOne.setPower(p3);
        //rightMotorTwo.setPower(p4);
    }

    public void resetWheels() {
        leftMotorOne.setPower(
                Range.clip(
                        -leftMotorOne.getCurrentPosition() / 100,
                        -Constants.MOTOR_MAX_SPEED,
                        Constants.MOTOR_MAX_SPEED
                )
        );

        leftMotorTwo.setPower(
                Range.clip(
                        -leftMotorTwo.getCurrentPosition() / 100,
                        -Constants.MOTOR_MAX_SPEED,
                        Constants.MOTOR_MAX_SPEED
                )
        );
    }

    public void stop() {
        leftMotorOne.setPower(0.0d);
        leftMotorTwo.setPower(0.0d);
        //rightMotorOne.setPower(0.0d);
        //rightMotorTwo.setPower(0.0d);
    }
}