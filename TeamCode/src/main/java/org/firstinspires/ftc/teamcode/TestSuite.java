package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.ai.cv.Camera;
import org.firstinspires.ftc.teamcode.subsystems.drives.SwerveDrive;

//Test Suite Code. Testing Swerve drive
@TeleOp(name="Test Suite", group="TeleOp")
public class TestSuite extends OpMode {
    private DcMotorSimple armMotor1;
    private DigitalChannel armMotorEncoder1;
    private int armPosition1 = 0;
    private boolean prevState1;
    private DcMotorSimple armMotor2;
    private DigitalChannel armMotorEncoder2;
    private int armPosition2 = 0;
    private boolean prevState2;

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive_front");
//        leftFrontDrive.setMode(RUN_USING_ENCODER);
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_drive_back");
//        leftBackDrive.setMode(RUN_USING_ENCODER);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        armMotor1 = hardwareMap.get(DcMotorSimple.class, "arm_drive_left");
//        armMotorEncoder1 = hardwareMap.get(DigitalChannel.class, "arm_motor_encoder_left");
//        prevState1 = armMotorEncoder1.getState();
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        armMotor2 = hardwareMap.get(DcMotorSimple.class, "arm_drive_right");
//        armMotorEncoder2 = hardwareMap.get(DigitalChannel.class, "arm_motor_encoder_right");
//        prevState2 = armMotorEncoder2.getState();
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {

        leftFrontDrive.setPower(Range.clip(gamepad1.left_stick_y, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));
        leftBackDrive.setPower(Range.clip(gamepad1.left_stick_y, -Constants.DRIVE_MOTOR_MAX_SPEED, Constants.DRIVE_MOTOR_MAX_SPEED));

        //telemetry.addData("LSY", gamepad1.left_stick_y);
//        armMotor1.setPower(Range.clip(gamepad1.left_stick_y,
//               -Constants.DRIVE_MOTOR_MAX_SPEED,
//               Constants.DRIVE_MOTOR_MAX_SPEED
//        ));
//        armMotor2.setPower(Range.clip(gamepad1.left_stick_y,
//                -Constants.DRIVE_MOTOR_MAX_SPEED,
//                Constants.DRIVE_MOTOR_MAX_SPEED
//        ));
//
//        if(armMotorEncoder1.getState() != prevState1) {
//            if(gamepad1.left_stick_y >= 0)
//                armPosition1++;
//            else
//                armPosition1--;
//        }
//        if(armMotorEncoder2.getState() != prevState2) {
//            if(gamepad1.left_stick_y >= 0)
//                armPosition2++;
//            else
//                armPosition2--;
//        }

//        prevState1 = armMotorEncoder1.getState();
//        telemetry.addData("Encoder Value", armPosition1);
//        telemetry.addData("Encoder Value", armPosition2);
    }

    @Override
    public void stop() {
//        armMotor1.setPower(0.0d);
//        armMotor2.setPower(0.0d);
    }
}
