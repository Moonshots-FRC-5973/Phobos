package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private Rev2mDistanceSensor distanceSensor;

    public DistanceSensor(HardwareMap hardwareMap, int id) {
        com.qualcomm.robotcore.hardware.DistanceSensor converter;
        converter = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "sensor_distance_" + id);
        distanceSensor = (Rev2mDistanceSensor)converter;
    }

    public double getDistance() {
        return getDistance(DistanceUnit.CM);
    }

    public double getDistance(DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }
}
