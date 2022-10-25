package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.sli.Vec2;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public class SwerveDrive {
    private DcMotor leftMotorOne;
    private DcMotor leftMotorTwo;
    private DcMotor rightMotorOne;
    private DcMotor rightMotorTwo;
    private int speed = 1;
    private boolean fieldCentric = true;
    private double lastLeftWheelError = 0;
    private double pidLeftIntegral = 0;
    private double lastRightWheelError = 0;
    private double pidRightIntegral = 0;
    private double lastUpdateTime = 0;
    private ElapsedTime timer;

    private IMU imu;

    public int getLeftEncodersDifference() {
        return leftMotorOne.getCurrentPosition() - leftMotorTwo.getCurrentPosition();
    }
    public double getLeftWheelAngle() {
        double angle = (360 * (getLeftEncodersDifference() % Constants.DRIVE_ENCODER_COUNTS_PER_REV) / Constants.DRIVE_ENCODER_COUNTS_PER_REV);
        if(fieldCentric) return angle + imu.getZAngle();
        return angle;
    }
    public int getRightEncodersDifference() {
        return rightMotorOne.getCurrentPosition() - rightMotorTwo.getCurrentPosition();
    }
    public double getRightWheelAngle() {
        double angle = (360 * (getRightEncodersDifference() % Constants.DRIVE_ENCODER_COUNTS_PER_REV) / Constants.DRIVE_ENCODER_COUNTS_PER_REV);
        if (fieldCentric) return angle + imu.getZAngle();
        return angle;
    }
    public void setSpeed(int speed) {
        this.speed = speed;
    }
    public double getSpeed() {
        return speed;
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
    public void upSpeed() {
        speed++;
    }
    public void dropSpeed() {
        speed--;
    }

    public SwerveDrive(HardwareMap hardwareMap, ElapsedTime timer) {
        leftMotorOne = hardwareMap.get(DcMotor.class, "left_drive_one");
        leftMotorOne.resetDeviceConfigurationForOpMode();

        leftMotorTwo = hardwareMap.get(DcMotor.class, "left_drive_two");
        leftMotorTwo.resetDeviceConfigurationForOpMode();

        rightMotorOne = hardwareMap.get(DcMotor.class, "right_drive_one");
        rightMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotorTwo = hardwareMap.get(DcMotor.class, "right_drive_two");
        rightMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = new IMU(hardwareMap);
        this.timer = timer;
    }

    public void drive(double forward, double turn) {
        drive(forward, 0.0d, turn);
    }

    public void drive(double forward, double strafe, double turn) {
        if(Math.abs(forward) <= Constants.DRIVE_INPUT_THRESHOLD) {
            forward = 0.0d;
        }
        if(Math.abs(strafe) <= Constants.DRIVE_INPUT_THRESHOLD) {
            strafe = 0.0d;
        }
        if(Math.abs(turn) <= Constants.DRIVE_INPUT_THRESHOLD) {
            turn = 0.0d;
        }

        if(turn != 0) {
            drive(
                    turn * getSpeed(),
                    turn * getSpeed(),
                    -turn * getSpeed(),
                    -turn * getSpeed()
            );
        } else {
            double targetAngle = Math.toDegrees(Math.atan2(forward, strafe));
        }

        double staticPower = Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2));
        lastUpdateTime = timer.seconds();

    }

    public void drive(double p1, double p2, double p3, double p4) {
        leftMotorOne.setPower(Range.clip(p1, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftMotorTwo.setPower(Range.clip(p2, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightMotorOne.setPower(Range.clip(p3, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightMotorTwo.setPower(Range.clip(p4, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
    }

    /*
    @input error The degrees off of the target the left wheel is
    @return An always positive, power offset value that ensures the wheel moves toward the target.
     */
    public double findLeftWheelPID(double error) {
        pidLeftIntegral += error / timer.seconds();
        double derivative = (error - lastLeftWheelError) / (timer.seconds() - lastUpdateTime);

        double pidOut = (Constants.DRIVE_PID_KP * error) +
                (Constants.DRIVE_PID_KI * pidLeftIntegral) +
                (Constants.DRIVE_PID_KD * derivative);

        lastLeftWheelError = error;
        return Math.abs(pidOut);
    }
    /*
    @input error The degrees off of the target the right wheel is
    @return An always positive, power offset value that ensures the wheel moves toward the target.
     */
    public double findRightWheelPID(double error) {
        pidRightIntegral += error / timer.seconds();
        double derivative = (error - lastRightWheelError) / (timer.seconds() - lastUpdateTime);

        double pidOut = (Constants.DRIVE_PID_KP * error) +
                (Constants.DRIVE_PID_KI * pidRightIntegral) +
                (Constants.DRIVE_PID_KD * derivative);

        lastRightWheelError = error;
        return Math.abs(pidOut);
    }

    public void turnWheelToAngle(double target) {
        double leftWheelDiff = getLeftWheelAngle() - target;
        double rightWheelDiff = getRightWheelAngle() - target;

        double leftRot = findLeftWheelPID(leftWheelDiff);
        double rightRot = findRightWheelPID(rightWheelDiff);

        drive(leftRot, -leftRot, rightRot, -rightRot);
    }
    public void resetWheels() {
        leftMotorOne.setPower(
                Range.clip(
                        -leftMotorOne.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        leftMotorTwo.setPower(
                Range.clip(
                        -leftMotorTwo.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        rightMotorOne.setPower(
                Range.clip(
                        -rightMotorOne.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );

        rightMotorTwo.setPower(
                Range.clip(
                        -rightMotorTwo.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                        -Constants.DRIVE_MOTOR_MAX_SPEED,
                        Constants.DRIVE_MOTOR_MAX_SPEED
                )
        );
    }
    public void stop() {
        leftMotorOne.setPower(0.0d);
        leftMotorTwo.setPower(0.0d);
        rightMotorOne.setPower(0.0d);
        rightMotorTwo.setPower(0.0d);
    }
}