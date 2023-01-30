//package org.firstinspires.ftc.teamcode.subsystems.sensors;
//
//import android.graphics.Color;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//@TeleOp(name="Color Sensor Test", group="AutonomousLeft")
//public class ColorSensorTest extends OpMode {
//    NormalizedColorSensor colorSensor;
//
//    public boolean isColorRed() {
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        telemetry.addData("Color Value",
//                "(%f, %f, %f, %f)", colors.red, colors.blue, colors.green, colors.alpha);
//        return false;
//    }
//
//    /**
//     * User defined init method
//     * <p>
//     * This method will be called once when the INIT button is pressed.
//     */
//    @Override
//    public void init() {
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class,"colors");
//    }
//
//    /**
//     * User defined loop method
//     * <p>
//     * This method will be called repeatedly in a loop while this op mode is running
//     */
//    @Override
//    public void loop() {
//        isColorRed();
//        telemetry.update();
//    }
//}
//
//
//


package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.sensors.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test Color")
public class TestColor extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Get the color sensor from hardwareMap
        color = new ColorSensor(hardwareMap, 1);
        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Intensity", color.getIntensity());
            telemetry.addData("Red", color.getRed());
            telemetry.addData("Green", color.getGreen());
            telemetry.addData("Blue", color.getBlue());
            telemetry.addData("Is Red", color.isRed());
            telemetry.addData("Is Green", color.isGreen());
            telemetry.addData("Is Blue", color.isBlue());
            telemetry.addData("Distance", color.getDistance());
            telemetry.update();
        }
    }
}