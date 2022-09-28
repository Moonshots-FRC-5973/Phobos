package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class ArcadeDrive {
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    private int check = 0;
    private double requestedRotation;

    public void init(HardwareMap hardwareMap) {
        leftDrive  = hardwareMap.get(DcMotor.class, Constants.DRIVE_MOTOR_ONE_NAME);
        rightDrive = hardwareMap.get(DcMotor.class, Constants.DRIVE_MOTOR_TWO_NAME);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void drive(double left, double right) {
        // Send calculated power to wheels
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    public void turnToDegree(double angleDelta) {
        requestedRotation = angleDelta;
    }

    private double notReallyPID() {
        // NOTE: Negative return values will increase the gyro's value
        double MAX_POWER = 0.7; // cap the power
        double MIN_POWER = 0.45; // lowest effective power
        int ENOUGH_CHECKS = 15; // how many times do we pass our target until we're satisfied?

        // determine the error
        double error = 0; // target - drive.gyro.getAngle();

        // determine the power output neutral of direction
        double output = Math.abs(error / requestedRotation) * MAX_POWER;
        if(output < MIN_POWER) output = MIN_POWER;
        if(output > MAX_POWER) output = MAX_POWER;

        // are we there yet? this is to avoid ping-ponging
        // plus we never stop the method unless our output is zero
        if(Math.abs(error) < Constants.DRIVE_ANGLE_TOLERANCE) check++;
        if(check > ENOUGH_CHECKS) return 0.0;

        // determine the direction
        // if I was trying to go a positive angle change from the start
        if(requestedRotation > 0){
            if(error > 0) return -output; // move in a positive direction
            else return output; // compensate for over-turning by going a negative direction
        }
        // if I was trying to go a negative angle from the start
        else{
            if(error < 0) return output; // move in a negative direction as intended
            else return -output; // compensate for over-turning by moving a positive direction
        }
    }

    public void stop(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
