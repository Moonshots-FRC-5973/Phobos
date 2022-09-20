package org.firstinspires.ftc.teamcode.subsystems.driveSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.v1.constants.drivesystem.TankDriveConstants;
import org.firstinspires.ftc.teamcode.v1.systems.sensors.Gyro;

public class TankDrive implements DriveSystem {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    //private Gyro gyro;


    public TankDrive(HardwareMap hardwareMap) {
        //leftMotor = hardwareMap.get(DcMotor.class, TankDriveConstants.LEFT_MOTOR_NAME);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        //rightMotor = hardwareMap.get(DcMotor.class, TankDriveConstants.RIGHT_MOTOR_NAME);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
      //  gyro = new Gyro(hardwareMap);
    }

    public void drive(double forward, double turn) {
        //if(Math.abs(forward) < TankDriveConstants.INPUT_THRESHOLD)
            forward = 0.0d;

        //if(Math.abs(turn) < TankDriveConstants.INPUT_THRESHOLD)
            turn = 0.0d;

        leftMotor.setPower(
                Range.clip(
                        (forward + turn),
                        -TankDriveConstants.MOTOR_MAXIMUM_POWER,
                        TankDriveConstants.MOTOR_MAXIMUM_POWER
                )
        );

        rightMotor.setPower(
                Range.clip(
                        (forward - turn),
                        -TankDriveConstants.MOTOR_MAXIMUM_POWER,
                        TankDriveConstants.MOTOR_MAXIMUM_POWER
                )
        );
    }

    public void turnToDegrees(double degrees) {
        int mult = 1;

        if(degrees - gyro.getZAngle() < 0) { // left
            mult = -1;
        } else if(degrees - gyro.getZAngle() > 0) { // right
            mult = 1;
        } else { // same
            return;
        }

        do {
            this.drive(0.0d,
                    Range.clip(mult * (gyro.getZAngle() - degrees) / 10,
                            -TankDriveConstants.MOTOR_MAXIMUM_POWER,
                            TankDriveConstants.MOTOR_MAXIMUM_POWER
                    )
            );
        } while(degrees >= mult * gyro.getZAngle());
    }

    public void close() {
        leftMotor.setPower(0.0d);
        rightMotor.setPower(0.0d);
    }
}
