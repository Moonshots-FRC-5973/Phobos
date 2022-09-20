package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.v1.constants.GyroConstants;
import org.firstinspires.ftc.teamcode.v1.constants.UnitConstants;
import org.firstinspires.ftc.teamcode.v1.systems.sli.Vec2;

public class Gyro {
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private static Vec2 location;

    public static Vec2 getLocation() {
        return location;
    }

    /**
     * Creates a new Gyro Object.
     * @param hardwareMap the HardwareMap of the active robot.
     */
    public Gyro(HardwareMap hardwareMap) {
        this(hardwareMap, new Vec2(0.0d, 0.0d));
    }

    public Gyro(HardwareMap hardwareMap, Vec2 startLocation) {
        if(startLocation != null)
            Gyro.location = startLocation;

        this.imu = hardwareMap.get(BNO055IMU.class, GyroConstants.DEVICE_NAME);
        this.parameters.angleUnit = UnitConstants.ANGLE_UNIT;
        this.parameters.accelUnit = UnitConstants.ACCEL_UNIT;

        if(!GyroConstants.CALIBRATION_FILE_NAME.equals("")) {
            this.parameters.loggingEnabled = true;
            this.parameters.calibrationDataFile = GyroConstants.CALIBRATION_FILE_NAME;
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
        return (double)(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, UnitConstants.ANGLE_UNIT.toAngleUnit()).firstAngle);
    }

    /**
     *
     * @return the Y angle of the internal IMU in the control panel.
     */
    public double getYAngle() {
        return (double)(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, UnitConstants.ANGLE_UNIT.toAngleUnit()).secondAngle);
    }

    /**
     *
     * @return the Z angle of the internal IMU in the control panel.
     */
    public double getZAngle() {
        return (double)(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, UnitConstants.ANGLE_UNIT.toAngleUnit()).thirdAngle);
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

    public void update(double deltaTime) {
        location.setX(location.getX() + (deltaTime * getXVelocity()));
        location.setY(location.getY() + (deltaTime * getZVelocity()));
    }
}
