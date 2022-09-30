package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.sli.Vec2;

public class SwerveDrive {
    private DcMotor leftMotorOne;
    private DcMotor leftMotorTwo;
    private DcMotor rightMotorOne;
    private DcMotor rightMotorTwo;
    private boolean isFieldCentric = true;

    public void makeFieldCentric() {
        isFieldCentric = true;
    }

    public void makeRobotCentric() {
        isFieldCentric = false;
    }

    public void switchMode() {
        isFieldCentric = !isFieldCentric;
    }

    public SwerveDrive(HardwareMap hardwareMap) {
        leftMotorOne = hardwareMap.get(DcMotor.class, "left_drive_one");
        leftMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotorTwo = hardwareMap.get(DcMotor.class, "left_drive_two");
        leftMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        rightMotorOne = hardwareMap.get(DcMotor.class, "right_drive_one");
        rightMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorTwo = hardwareMap.get(DcMotor.class, "right_drive_two");
        rightMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

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

        if(isFieldCentric) {
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

    public void stop() {
        leftMotorOne.setPower(0.0d);
        leftMotorTwo.setPower(0.0d);
        //rightMotorOne.setPower(0.0d);
        //rightMotorTwo.setPower(0.0d);
    }
}
