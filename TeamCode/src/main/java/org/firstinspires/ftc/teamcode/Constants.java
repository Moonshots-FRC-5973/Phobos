package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

public class Constants {
    // _______________
    // ---  DRIVE  ---
    public static final double DRIVE_ANGLE_TOLERANCE = 1;
    public static final String DRIVE_MOTOR_ONE_NAME = "left_drive";
    public static final String DRIVE_MOTOR_TWO_NAME = "right_drive";
    public static final double ENCODER_COUNTS_PER_REV = 1000;
    public static final double INPUT_THRESHOLD = 0.15;
    public static final double MOTOR_MAX_SPEED = 0.75;

    // ________________
    // ---  SERVOS  ---
    public static final String ARM_SERVO_NAME = "servo0";

    // ________________
    // ---  SENSOR  ---
    public static final String GYRO_NAME = "imu";
    public static final String LOG_FILE_NAME = "gyro.log";
    public static final String CALIBRATION_FILE_NAME = "gyro_calibration.log";

    public static final BNO055IMU.AccelUnit ACCEL_UNIT = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    public static final BNO055IMU.AngleUnit ANGLE_UNIT = BNO055IMU.AngleUnit.DEGREES;
}
