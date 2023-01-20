package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.sensors.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Main")
public class Autonomous extends LinearOpMode {

    private static double TILE_DIST = 22;
    private static double FAR_DIST = 48;
    private static double NEAR_DIST = 4;
    private static double MOTOR_SPEED = 0.1;
    private static double ROT_ANGLE_3 = 35;

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
        drivetrain = new MecanumDrive(hardwareMap, timeyMcTimeTimerson, telemetry);
        drivetrain.toggleFieldCentric();
        clawyMcClawClawferson = new Claw(hardwareMap, telemetry);
        colorMcColorColorson = new ColorSensor(hardwareMap, 1);
        // back
        distanceBack = new DistanceSensor(hardwareMap, 0);
        // left
        distanceLeft = new DistanceSensor(hardwareMap, 1);
        // right
        distanceRight = new DistanceSensor(hardwareMap, 2);

        // we were misreading the field so double polling :)
        double right1, right2, left1, left2;
        do {

            right1 = distanceRight.getDistance();
            left1 = distanceLeft.getDistance();

            sleep(1000);

            right2 = distanceRight.getDistance();
            left2 = distanceLeft.getDistance();
        }
        // checking for dif between 1st + 2nd reading
        while (Math.abs(right1 - right2) > 5 && Math.abs(left1 - left2) > 5);

        if(right1 > left1) {
            onLeftSide = true;
        } else {
            onLeftSide = false;
        }

        telemetry.addData("On Left Side", onLeftSide);
        telemetry.update();

        waitForStart();

        if(!opModeIsActive()) {
            return;
        }

        findTarget();

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
                drivetrain.stop();
        }
        /**
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


        }
         */
    }

    private void case1() {
        while(distanceBack.getDistance(DistanceUnit.INCH) <= TILE_DIST)
            drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);

        if(onLeftSide) {
            while(distanceLeft.getDistance(DistanceUnit.INCH) >= NEAR_DIST)
                drivetrain.drive(0.0d, -MOTOR_SPEED, 0.0d);
        } else {
            while(distanceRight.getDistance(DistanceUnit.INCH) <= FAR_DIST)
                drivetrain.drive(0.0d, -MOTOR_SPEED, 0.0d);
        }

        drivetrain.stop();
    }

    private void case2() {
        while(distanceBack.getDistance(DistanceUnit.INCH) <= TILE_DIST)
            drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);

        drivetrain.stop();
    }

    private void case3() {
        while(distanceBack.getDistance(DistanceUnit.INCH) <= TILE_DIST + 5)
            drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);

        while(distanceBack.getDistance(DistanceUnit.INCH) >= TILE_DIST + 3)
            drivetrain.drive(-MOTOR_SPEED, 0.0d, 0.0d);

        drivetrain.stop();

        if(onLeftSide) {
            while(distanceLeft.getDistance(DistanceUnit.INCH) <= FAR_DIST)
                drivetrain.drive(0.0d, MOTOR_SPEED, 0.0d);
            while(Math.abs(drivetrain.getIMU().getZAngle() + ROT_ANGLE_3) > Constants.DRIVE_ANGLE_TOLERANCE) {
                drivetrain.turnRobotToAngle(-ROT_ANGLE_3);
            } // Stop it from freaking out now that its turned and the distance sensor have triggering data - you know who wrote this
        } else {
            while(distanceRight.getDistance(DistanceUnit.INCH) >= NEAR_DIST)
                drivetrain.drive(0.0d, MOTOR_SPEED, 0.0d);
        }

        drivetrain.stop();

        clawyMcClawClawferson.setHigh();
        sleep(3000);
        clawyMcClawClawferson.angleClaw();
        drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);
        sleep(2200);
        drivetrain.stop();
        drivetrain.drive(-MOTOR_SPEED, 0.0d, 0.0d);
        sleep(1900);
        drivetrain.stop();
        clawyMcClawClawferson.open();
        sleep(250);
        drivetrain.drive(-MOTOR_SPEED, 0.0d, 0.0d);
        clawyMcClawClawferson.close();
        sleep(600);
        clawyMcClawClawferson.setMin();
        while(Math.abs(drivetrain.getIMU().getZAngle()) > 1) {
            drivetrain.turnRobotToAngle(0);
        } // Stop it from freaking out now that its turned and the distance sensor have triggering data - you know who wrote this
        drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);
        sleep(1100);
        drivetrain.stop();
    }

    private void findTarget() {
        while(colorMcColorColorson.getIntensity() <= 200) {
            drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);
        }

        drivetrain.stop();
        whenDetermined = timeyMcTimeTimerson.seconds();
        if (colorMcColorColorson.isRed()) {
            targetMcTargetTargetson = 3;
        } else if (colorMcColorColorson.isGreen()) {
            targetMcTargetTargetson = 2;
        } else if (colorMcColorColorson.isBlue()) {
            targetMcTargetTargetson = 1;
        }
    }
}
