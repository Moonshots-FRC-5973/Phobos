package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.sensors.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Main")
public class Autonomous extends LinearOpMode {

    private static double TILE_DIST = 22;

    private Drivetrain drivetrain;
    private ElapsedTime timeyMcTimeTimerson = new ElapsedTime();
    private Claw clawyMcClawClawferson;
    private ColorSensor colorMcColorColorson;
    private DistanceSensor distanceBack;
    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;
    private int targetMcTargetTargetson = 0;
    private double whenDetermined = 0;
    private boolean onLeftSide = false;
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrive(hardwareMap, timeyMcTimeTimerson, null);
        clawyMcClawClawferson = new Claw(hardwareMap, telemetry);
        colorMcColorColorson = new ColorSensor(hardwareMap, 1);
        // back
        distanceBack = new DistanceSensor(hardwareMap, 0);
        // left
        distanceLeft = new DistanceSensor(hardwareMap, 1);
        // right
        distanceRight = new DistanceSensor(hardwareMap, 2);

        if(distanceRight.getDistance() > distanceLeft.getDistance()) {
            onLeftSide = true;
        } else {
            onLeftSide = false;
        }

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Intensity", colorMcColorColorson.getIntensity());
            telemetry.addData("Red", colorMcColorColorson.getRed());
            telemetry.addData("Green", colorMcColorColorson.getGreen());
            telemetry.addData("Blue", colorMcColorColorson.getBlue());
            telemetry.addData("Back", distanceBack.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left", distanceLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right", distanceRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("On Left Side", onLeftSide);
            telemetry.update();

            switch(targetMcTargetTargetson) {
                case 1:
                    case1();
                    break;
                case 2:
                    case2();
                    break;
                case 3:
                    case3();
                    break;
                default:
                    findTarget();
            }
        }
    }

    private void case1() {
        while(distanceBack.getDistance(DistanceUnit.INCH) <= TILE_DIST)
            drivetrain.drive(0.1d, 0.0d, 0.0d);

        if(onLeftSide) {
            while(distanceLeft.getDistance(DistanceUnit.INCH) >= 4)
                drivetrain.drive(0.0d, -0.1d, 0.0d);
        } else {
            while(distanceRight.getDistance(DistanceUnit.INCH) <= 48)
                drivetrain.drive(0.0d, -0.1d, 0.0d);
        }

        drivetrain.stop();
    }

    private void case2() {
        while(distanceBack.getDistance(DistanceUnit.INCH) <= TILE_DIST)
            drivetrain.drive(0.1d, 0.0d, 0.0d);

        drivetrain.stop();
    }

    private void case3() {
        while(distanceBack.getDistance(DistanceUnit.INCH) <= TILE_DIST)
            drivetrain.drive(0.1d, 0.0d, 0.0d);

        if(onLeftSide) {
            while(distanceLeft.getDistance(DistanceUnit.INCH) <= 48)
                drivetrain.drive(0.0d, 0.1d, 0.0d);
        } else {
            while(distanceRight.getDistance(DistanceUnit.INCH) >= 4)
                drivetrain.drive(0.0d, 0.1d, 0.0d);
        }

        drivetrain.stop();
    }

    private void findTarget() {
        if(colorMcColorColorson.getIntensity() >= 200) {
            drivetrain.stop();
            whenDetermined = timeyMcTimeTimerson.seconds();
            if(colorMcColorColorson.isRed()) {
                targetMcTargetTargetson = 3;
            } else if(colorMcColorColorson.isGreen()) {
                targetMcTargetTargetson = 2;
            } else if(colorMcColorColorson.isBlue()) {
                targetMcTargetTargetson = 1;
            }
        } else {
            drivetrain.drive(0.1d, 0.0d, 0.0d);
        }
    }
}
