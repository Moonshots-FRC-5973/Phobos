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

    private static double RANGE_MIN = 23;

    private Drivetrain driveyMcDriveDriverson;
    private ElapsedTime timeyMcTimeTimerson = new ElapsedTime();
    private Claw clawyMcClawClawferson;
    private ColorSensor colorMcColorColorson;
    private DistanceSensor distanceMcDistanceDistancesonSr;
    private DistanceSensor distanceMcDistanceDistancesonJr;
    private int targetMcTargetTargetson = 0;
    private double whenDetermined = 0;
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
        driveyMcDriveDriverson = new MecanumDrive(hardwareMap, timeyMcTimeTimerson, null, false);
        clawyMcClawClawferson = new Claw(hardwareMap, telemetry);
        colorMcColorColorson = new ColorSensor(hardwareMap, 1);
        distanceMcDistanceDistancesonSr = new DistanceSensor(hardwareMap, 0);
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Intensity", colorMcColorColorson.getIntensity());
            telemetry.addData("Red", colorMcColorColorson.getRed());
            telemetry.addData("Green", colorMcColorColorson.getGreen());
            telemetry.addData("Blue", colorMcColorColorson.getBlue());
            telemetry.update();

            switch(targetMcTargetTargetson) {
                case 1:
                    if (distanceMcDistanceDistancesonSr.getDistance(DistanceUnit.INCH) <= RANGE_MIN)
                        driveyMcDriveDriverson.drive(0.1d, 0.0d, 0.0d);
                    else
                        driveyMcDriveDriverson.drive(0.0d, -0.1d, 0.0d);
                    break;
                case 2:
                    if (distanceMcDistanceDistancesonSr.getDistance(DistanceUnit.INCH) <= RANGE_MIN)
                        driveyMcDriveDriverson.drive(0.1d, 0.0d, 0.0d);
                    else
                        driveyMcDriveDriverson.stop();
                    break;
                case 3:
                    if (distanceMcDistanceDistancesonSr.getDistance(DistanceUnit.INCH) <= RANGE_MIN)
                        driveyMcDriveDriverson.drive(0.1d, 0.0d, 0.0d);
                    else
                        driveyMcDriveDriverson.drive(0.0d, 0.1d, 0.0d);
                    break;
                default:
                    if(colorMcColorColorson.getIntensity() >= 200) {
                        driveyMcDriveDriverson.stop();
                        whenDetermined = timeyMcTimeTimerson.seconds();
                        if(colorMcColorColorson.isRed()) {
                            targetMcTargetTargetson = 3;
                        } else if(colorMcColorColorson.isGreen()) {
                            targetMcTargetTargetson = 2;
                        } else if(colorMcColorColorson.isBlue()) {
                            targetMcTargetTargetson = 1;
                        }
                    } else {
                        driveyMcDriveDriverson.drive(0.1d, 0.0d, 0.0d);
                        telemetry.addData("Drive", "Forward");
                    }
            }
        }
    }
}
