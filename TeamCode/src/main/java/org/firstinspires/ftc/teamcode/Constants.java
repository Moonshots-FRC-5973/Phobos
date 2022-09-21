package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

public class Constants {
    // _______________
    // ---  DRIVE  ---
    public static final double ANGLE_TOLERANCE = 1;
    // ________________
    // ---  SERVOS  ---
    public static final String ARM_SERVO_NAME = "servo0";
    // ________________
    // ---  SENSOR  ---
    public static final String GYRO_NAME = "gyro";
    public static final String LOG_FILE_NAME = "gyro.log";
    public static final String CALIBRATION_FILE_NAME = "file.file";

    public static final BNO055IMU.AccelUnit ACCEL_UNIT = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    public static final BNO055IMU.AngleUnit ANGLE_UNIT = BNO055IMU.AngleUnit.DEGREES;
}
