package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.sensors.ColorSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Main")
public class Autonomous extends LinearOpMode {

    private Drivetrain driveyMcDriveDriverson;
    private ElapsedTime timeyMcTimeTimerson = new ElapsedTime();
    private Claw clawyMcClawClawferson;
    private ColorSensor colorMcColorColorson;
    private int targetMcTargetTargetson = 0;

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
        driveyMcDriveDriverson = new MecanumDrive(hardwareMap, timeyMcTimeTimerson, telemetry);
        clawyMcClawClawferson = new Claw(hardwareMap, telemetry);
        colorMcColorColorson = new ColorSensor(hardwareMap, 1);
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Intensity", colorMcColorColorson.getIntensity());
            telemetry.addData("Red", colorMcColorColorson.getRed());
            telemetry.addData("Green", colorMcColorColorson.getGreen());
            telemetry.addData("Blue", colorMcColorColorson.getBlue());
            telemetry.update();
            switch(targetMcTargetTargetson) {
                case 1:
                    driveyMcDriveDriverson.drive(0.0d, -0.1d, 0.0d);
                case 2:
                    driveyMcDriveDriverson.stop();
                case 3:
                    driveyMcDriveDriverson.drive(0.0d, 0.1d, 0.0d);
                default:
                    if(colorMcColorColorson.getIntensity() >= 60) {
                        driveyMcDriveDriverson.stop();
                        if(colorMcColorColorson.isRed()) {
                            targetMcTargetTargetson = 1;
                        } else if(colorMcColorColorson.isGreen()) {
                            targetMcTargetTargetson = 2;
                        } else if(colorMcColorColorson.isBlue()) {
                            targetMcTargetTargetson = 3;
                        }
                    } else {
                        driveyMcDriveDriverson.drive(-0.15d, 0.0d, 0.0d);
                        telemetry.addData("Drive", "Forward");
                    }
            }
        }
    }
}
