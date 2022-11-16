package org.firstinspires.ftc.teamcode.subsystems.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.IMU;

public abstract class Drivetrain {
    protected Telemetry telemetry;
    public Telemetry getTelemetry() {
        return telemetry;
    }

    protected ElapsedTime runtime;
    public ElapsedTime getRuntime() {
        return runtime;
    }

    protected IMU imu;
    public IMU getIMU() {
        return imu;
    }

    public double speed = 1;
    public boolean isFieldCentric = true;

    public Drivetrain(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        this.imu = new IMU(hardwareMap);
        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    public abstract void drive(double forward, double strafe, double turn);
    public abstract void drive(double m1, double m2, double m3, double m4);
    public abstract void resetWheels();
    public abstract void turnRobotToAngle(double target);
    public void stop() {
        drive(0.0d, 0.0d, 0.0d, 0.0d);
    }
}
