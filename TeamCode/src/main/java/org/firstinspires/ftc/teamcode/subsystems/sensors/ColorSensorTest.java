//package org.firstinspires.ftc.teamcode.subsystems.sensors;
//
//import android.graphics.Color;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//@TeleOp(name="Color Sensor Test", group="Autonomous")
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


package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class ColorSensorTest extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;
    DcMotor myMotor;
    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");
        myMotor = hardwareMap.get(DcMotor.class,"right_drive_one");
        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {


            myMotor.setPower(.75);
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }
    }
}