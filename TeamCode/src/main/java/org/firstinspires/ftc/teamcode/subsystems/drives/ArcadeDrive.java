package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class ArcadeDrive extends Drivetrain {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private double targetAngle;

    public ArcadeDrive(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
    }

    @Override
    public void drive(double forward, double strafe, double turn) {
        drive(forward + turn, forward - turn, 0.0d, 0.0d);
    }

    @Override
    public void drive(double m1, double m2, double m3, double m4) {
        leftDrive.setPower(Range.clip(m1, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        rightDrive.setPower(Range.clip(m2, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
    }

    @Override
    public void resetWheels() {
        drive(-imu.getZAngle() / 100, imu.getZAngle() / 100, 0.0d, 0.0d);
    }

    @Override
    public void turnRobotToAngle(double target) {

    }
}
