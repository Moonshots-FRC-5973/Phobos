package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.constants.GyroConstants;

public class IMU {
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public IMU(HardwareMap hardwareMap) {

        this.imu = hardwareMap.get(BNO055IMU.class, GyroConstants.DEVICE_NAME);
        this.parameters.angleUnit = GyroConstants.ANGLE_UNIT;
        this.parameters.accelUnit = GyroConstants.ACCEL_UNIT;

        if(!GyroConstants.LOG_FILE_NAME.equals("")) {
            this.parameters.loggingEnabled = true;
            this.parameters.calibrationDataFile = GyroConstants.LOG_FILE_NAME;
        } else {
            this.parameters.loggingEnabled = false;
        }

        this.imu.initialize(this.parameters);
    }

    /**
     *
     * @return the X angle of the internal IMU in the control panel.
     */
    public double getXAngle() {
        return (double)(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, GyroConstants.ANGLE_UNIT.toAngleUnit()).firstAngle);
    }

    /**
     *
     * @return the Y angle of the internal IMU in the control panel.
     */
    public double getYAngle() {
        return (double)(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, GyroConstants.ANGLE_UNIT.toAngleUnit()).secondAngle);
    }

    /**
     *
     * @return the Z angle of the internal IMU in the control panel.
     */
    public double getZAngle() {
        return (double)(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, GyroConstants.ANGLE_UNIT.toAngleUnit()).thirdAngle);
    }

    /**
     *
     * @return a double array, ordered XYZ, of the angle.
     */
    public double[] getAngle() {
        double[] out = new double[3];

        out[0] = getXAngle();
        out[1] = getYAngle();
        out[2] = getZAngle();

        return out;
    }

    /**
     *
     * @return The X axis velocity of the control panel.
     */
    public double getXVelocity() {
        return (double)(imu.getVelocity().xVeloc);
    }

    /**
     *
     * @return The Y axis velocity of the control panel.
     */
    public double getYVelocity() {
        return (double)(imu.getVelocity().yVeloc);
    }

    /**
     *
     * @return The Z axis velocity of the control panel.
     */
    public double getZVelocity() {
        return (double)(imu.getVelocity().zVeloc);
    }

    /**
     *
     * @return An ordered XYZ array of the control panel's current velocity.
     */
    public double[] getVelocity() {
        double[] out = new double[3];

        out[0] = getXVelocity();
        out[1] = getYVelocity();
        out[2] = getZVelocity();

        return out;
    }
}