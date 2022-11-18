package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive extends Drivetrain {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private boolean driving = true;

    public MecanumDrive(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive_front");
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_drive_back");
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_drive_back");
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void drive(double forward, double strafe, double turn) {
        if(Math.abs(forward) <= Constants.DRIVE_INPUT_THRESHOLD) {
            forward = 0.0d;
        }

        if(Math.abs(strafe) <= Constants.DRIVE_INPUT_THRESHOLD) {
            strafe = 0.0d;
        }

        if (Math.abs(turn) >= Constants.DRIVE_INPUT_THRESHOLD) {
            drive(turn, turn, -turn, turn);
            return;
        }

        drive(
              -forward - strafe,
              -forward + strafe,
              -forward + strafe,
              forward + strafe
        );
    }

    public void drive(double leftFront, double leftRear, double rightFront, double rightRear) {
        if(telemetry != null) {
            telemetry.addData("leftFront", leftFront);
            telemetry.addData("leftRear", leftRear);
            telemetry.addData("rightFront", rightFront);
            telemetry.addData("rightRear", rightRear);
         }

        leftFrontDrive.setPower(Range.clip(leftFront, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftBackDrive.setPower(Range.clip(leftRear, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightFrontDrive.setPower(Range.clip(rightFront, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightBackDrive.setPower(Range.clip(rightRear, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
    }

    @Override
    public void resetWheels() {
        drive(
                -leftFrontDrive.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                -leftBackDrive.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                -rightFrontDrive.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV,
                -rightBackDrive.getCurrentPosition() / Constants.DRIVE_ENCODER_COUNTS_PER_REV
        );
    }

    @Override
    public boolean turnRobotToAngle(double target) {
        // NOTE: Negative return values will increase the gyro's value
        double MAX_POWER = 0.5; // cap the power
        double MIN_POWER = 0.1; // lowest effective power
        int ENOUGH_CHECKS = 15; // how many times do we pass our target until we're satisfied?
        int check = 0;

        // determine the error
        double error = target - imu.getZAngle();

        // determine the power output neutral of direction
        double output = Math.abs(error / target) * MAX_POWER;
        if (output < MIN_POWER) output = MIN_POWER;
        if (output > MAX_POWER) output = MAX_POWER;

        //we are commenting this out because we got rid of the loop.
        // are we there yet? this is to avoid ping-ponging
        // plus we never stop the method unless our output is zero
//        if (Math.abs(error) < Constants.DRIVE_ANGLE_TOLERANCE) check++;
//        if (check > ENOUGH_CHECKS) {
//            stop();
//            break;
//        }

        // determine the direction
        // if I was trying to go a positive angle change from the start
        if (target > 0) {
            if (error > 0) drive(output, output, -output, output); // move in a positive direction
            else drive(-output, -output, output, -output); // compensate for over-turning by going a negative direction
        }
        // if I was trying to go a negative angle from the start
        else {
            if (error < 0) drive(-output, -output, output, -output); // move in a negative direction as intended
            else drive(output, output, -output, output); // compensate for over-turning by moving a positive direction
        }
        //if we have reached our angle we are done
        if (math.abs(error < Constants.DRIVE_ANGLE_TOLERANCE)){
            stop();
            return true;
        }
        else {
            return false;
        }
    }

    public void turnRobotByDegree(double target){
        turnRobotToAngle(imu.getZAngle() + target);
    }
}
