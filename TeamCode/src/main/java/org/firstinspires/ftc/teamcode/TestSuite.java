package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.sensors.DistanceSensor;

@TeleOp(group="TeleOp", name="Test Suite")
public class TestSuite extends LinearOpMode {
    private ColorSensor colorSensor;
    private DistanceSensor distanceBack;

    @Override
    public void runOpMode() {
        colorSensor = new ColorSensor(hardwareMap, 1.0f);
        distanceBack = new DistanceSensor(hardwareMap, 0);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Color", "(" + colorSensor.isRed() + ", " + colorSensor.isGreen() + ", " + colorSensor.isBlue() + ")");
            telemetry.addData("Intensity", colorSensor.getIntensity());
            telemetry.addData("Back Distance", distanceBack.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
