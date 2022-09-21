//package org.firstinspires.ftc.teamcode.subsystems.sensors;
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.Constants;
//
//
//public class DistanceSensor {
//    private static com.qualcomm.robotcore.hardware.DistanceSensor converter;
//
//    private Rev2mDistanceSensor distanceSensor;
//
//    public DistanceSensor(HardwareMap hardwareMap, int sensor_number) {
//        converter = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class,
//                Constants.DEVICE_PREFIX + sensor_number);
//        distanceSensor = (Rev2mDistanceSensor)converter;
//    }
//
//    public double getDistance() {
//        return distanceSensor.getDistance(Constants.DISTANCE_UNIT);
//    }
//
//    public String getDeviceName() {
//        return distanceSensor.getDeviceName();
//    }
//}
