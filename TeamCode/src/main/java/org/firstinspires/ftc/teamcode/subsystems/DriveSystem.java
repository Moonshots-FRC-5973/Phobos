package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.wrappers.DriveSystemBase;

public class DriveSystem {
    private DriveSystemBase driveSystem;

    public DriveSystem(Class<? extends DriveSystemBase> systemBase) throws Exception {
        this.driveSystem = systemBase.newInstance();
    }
}
