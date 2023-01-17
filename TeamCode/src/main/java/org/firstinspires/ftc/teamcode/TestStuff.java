package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.sensors.DistanceSensor;

import java.util.List;

//Test Suite Code.
@TeleOp(name="Test Suite", group="TeleOp")
public class TestStuff extends OpMode {


    private ColorSensor colorMcColorColorson;
    private DistanceSensor distanceBack;
    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;

    @Override
    public void init() {
        colorMcColorColorson = new ColorSensor(hardwareMap, 1);
        // back
        distanceBack = new DistanceSensor(hardwareMap, 0);
        // left
        distanceLeft = new DistanceSensor(hardwareMap, 1);
        // right
        distanceRight = new DistanceSensor(hardwareMap, 2);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Color", "Color(" + colorMcColorColorson.getRed() + ", " + colorMcColorColorson.getGreen() + ", ", colorMcColorColorson.getBlue() + ")");
        telemetry.addData("Color", "Distance " + colorMcColorColorson.getDistance());
        telemetry.addData("Back", distanceBack.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left", distanceLeft.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right", distanceRight.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    public void start() {

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
    }

    @Override
    public void stop() {

    }
}
