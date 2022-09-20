package org.firstinspires.ftc.teamcode.subsystems.driveSystem;

public interface DriveSystem {
    void drive(double forward, double turn);
    void turnToDegrees(double degrees);
    void close();
}
