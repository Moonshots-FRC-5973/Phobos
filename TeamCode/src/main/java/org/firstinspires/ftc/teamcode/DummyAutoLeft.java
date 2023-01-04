package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drives.MecanumDrive;

@Autonomous(name = "Dummy Auto Left")
public class DummyAutoLeft extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive;
    private Claw clawyMcClawClawferson;

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
        clawyMcClawClawferson = new Claw(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, runtime, telemetry);

        waitForStart();
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() <= 1) {
            drive.drive(0.0d, -0.5d, 0.0d);
        }
    }
}
