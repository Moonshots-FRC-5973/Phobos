package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.sensors.DistanceSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Right")
public class AutonomousRight extends LinearOpMode {

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
    private boolean onLeftSide;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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

        onLeftSide = false;

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
        drivetrain.stop();

        clawyMcClawClawferson.setMin();
        clawyMcClawClawferson.angleClaw();

        sleep(30000);

    }

    private void case1() {
        while(distanceBack.getDistance(DistanceUnit.INCH) <= TILE_DIST + 5)
            drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);

        while(distanceBack.getDistance(DistanceUnit.INCH) >= TILE_DIST + 3)
            drivetrain.drive(-MOTOR_SPEED, 0.0d, 0.0d);

        drivetrain.stop();

        while(distanceRight.getDistance(DistanceUnit.INCH) <= FAR_DIST)
            drivetrain.drive(0.0d, -MOTOR_SPEED, 0.0d);
    }

    private void case2() {
        while(distanceBack.getDistance(DistanceUnit.INCH) <= TILE_DIST + 7)
            drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);

        while(distanceBack.getDistance(DistanceUnit.INCH) >= TILE_DIST + 3)
            drivetrain.drive(-MOTOR_SPEED, 0.0d, 0.0d);
    }

    private void case3() {
        while(distanceBack.getDistance(DistanceUnit.INCH) <= TILE_DIST + 5)
            drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);

        while(distanceBack.getDistance(DistanceUnit.INCH) >= TILE_DIST + 3)
            drivetrain.drive(-MOTOR_SPEED, 0.0d, 0.0d);

        drivetrain.stop();

        while(distanceRight.getDistance(DistanceUnit.INCH) >= NEAR_DIST)
            drivetrain.drive(0.0d, MOTOR_SPEED, 0.0d);
    }

    private void findTarget() {
        while(colorMcColorColorson.getIntensity() <= 200) {
            drivetrain.drive(MOTOR_SPEED, 0.0d, 0.0d);
        }

        drivetrain.stop();
        whenDetermined = timeyMcTimeTimerson.seconds();
        if (colorMcColorColorson.isRed()) {
            targetMcTargetTargetson = 3;
            telemetry.addData("Side", "Red: 3");
        } else if (colorMcColorColorson.isGreen()) {
            targetMcTargetTargetson = 2;
            telemetry.addData("Side", "Green: 2");
        } else if (colorMcColorColorson.isBlue()) {
            targetMcTargetTargetson = 1;
            telemetry.addData("Side", "Blue: 1");
        }
        telemetry.update();
    }
}
