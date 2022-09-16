package org.firstinspires.ftc.teamcode.subsystems;

public class DriveSystem {
    private DriveSystemBase driveSystem;

    public DriveSystem(Class<? extends DriveSystemBase> systemBase) throws Exception {
        switch (systemBase) {
            case TankDrive.class:
                break;
            default:
                throw new Exception("DriveSystem cannot accept this class type!");
        }
    }
}
