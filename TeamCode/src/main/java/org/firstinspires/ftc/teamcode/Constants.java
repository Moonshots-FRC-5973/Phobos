package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

public class Constants {

    // _______________
    // ---  DRIVE  ---
    // Precision of the subsystem approaching an angle
    public static final double DRIVE_ANGLE_TOLERANCE = 1;
    // motor names
    public static final String DRIVE_MOTOR_ONE_NAME = "left_drive";
    public static final String DRIVE_MOTOR_TWO_NAME = "right_drive";
    //Swerve drive encoder count difference for the wheel to rotate once fully
    public static final double DRIVE_ENCODER_COUNTS_PER_REV = 1023.568;
    // DEPRECATED: Power multiplier for the wheel to rotate. use Constants.DRIVE_PID_KP instead
    //@Deprecated
    public static final double DRIVE_ROTATION_POWER_MODIFIER = 1 / Math.sin(Math.toRadians(20));
    // Minimum input for the subsystem to respond
    public static final double DRIVE_INPUT_THRESHOLD = 0.15;
    public static final double DRIVE_MOTOR_MAX_SPEED = 0.75;
    public static final double DRIVE_PID_KP = 1 / 45;
    public static final double DRIVE_PID_KI = 1;
    public static final double DRIVE_PID_KD = 1;

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

    // ________________
    // OBJECT_DETECTION
    public static final String OD_VUFORIA_KEY = "AS6a0rf/////AAABmQTWiptCrE+LufdlUzBT8weOkDKTan22xYq7kRmbpkAd2B1wy+uBaVuTdp4ngclG4NG6WQ8+8+nnRd+v6OB5Gzm+jMMh02iC+WrML6/2ArgWlM1vh43nEyfKOaOyJ4uZYqKMNAEcXqNLKK2+PdtmQQgiwGhna/VKV/Qdkhwsxt6w+4VGETJJwxT8k+tXTal2DGF5Sr9c69Lz0O0drCDZ2+ZUtuhOn1X+dkVoGxAoqSh/sYiqssxEtfGaf551TQytAXNBpbMgYXNGRSR6WAke2lVC4BxEowhiacPiZDLOZgVrHPc0bJbtN2kIF3OWk/FHj3tuHQ6seHZR4cU/6S7AeP3PaBwnYKbFvg8svUAy3vxD";
    public static final String OD_RES_MAIN_TFLITE = "res/main.tflite";
    public static final String[] OD_LABELS = {
            ""
    };
}
