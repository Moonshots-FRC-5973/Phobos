/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drives.MecanumDrive;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Phobos Main", group="TeleOp")
public class Phobos extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // ----------
    // SUBSYSTEMS
    private Drivetrain drive;
    private Claw clawyMcClawClawferson;

    // Input state holders
    private boolean gp2bPressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Add the telemetry output to the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // INIT SUBSYSTEMS
        clawyMcClawClawferson = new Claw(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, runtime, telemetry);

        //Send the telemetry info pieces to the DS / Dashboard
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Runtime", runtime.seconds());

        // DRIVE CONTROLS
        // Comment out the below line to not have the robot drive around.
        driver1Inputs();

        // CLAW CONTROLS
        driver2Inputs();

        telemetry.addData("ArmLeft", clawyMcClawClawferson.getLeftCurrentHeight());
        telemetry.addData("ArmRight", clawyMcClawClawferson.getRightCurrentHeight());
        telemetry.addData("IMU", "(" + drive.getIMU().getXAngle() + ", " + drive.getIMU().getYAngle() + ", " + drive.getIMU().getZAngle() + ")");
        telemetry.update();
    }

    /**
     * gamepad1: responsible for the drive System, movement, control speed, etc.
     */
    private void driver1Inputs() {
        if (gamepad1.left_stick_button) {
            telemetry.addData("Drive", "Resetting wheels");
            drive.resetWheels();
            return;
        }

        // DPad inputs, checking for overload; control for the drivetrain to rotate the robot
        boolean turnUp = (gamepad1.dpad_up && !gamepad1.dpad_down);
        boolean turnDown = (gamepad1.dpad_down && !gamepad1.dpad_up);
        boolean turnLeft = (gamepad1.dpad_left && !gamepad1.dpad_right);
        boolean turnRight = (gamepad1.dpad_right && !gamepad1.dpad_left);

        if(turnUp) {
            telemetry.addData("Drive", "Turning back to original front");
            if(turnRight) {
                drive.turnRobotToAngle(-45);
            } else if(turnLeft) {
                drive.turnRobotToAngle(45);
            } else {
                drive.turnRobotToAngle(0);
            }
        } else if(turnDown) {
            if (turnRight) {
                drive.turnRobotToAngle(-135);
            } else if (turnLeft) {
                drive.turnRobotToAngle(135);
            } else {
                telemetry.addData("Drive", "Turning to the reverse of start");
                drive.turnRobotToAngle(180);
            }
        } else if(turnRight) {
            drive.turnRobotToAngle(-90);
        } else if(turnLeft) {
            drive.turnRobotToAngle(90);
        }
        else {
            telemetry.addData("Drive", "Listening to LSX, LSY, RSX");
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        if(gamepad1.a) {
            drive.toggleFieldCentric();
        }
    }

    /**
     * gamepad2: Responsible for claw and vision processing controls
     */
    private void driver2Inputs() {
        if(gamepad2.dpad_up) {
            clawyMcClawClawferson.setHigh();
        } else if(gamepad2.dpad_left) {
            clawyMcClawClawferson.setMid();
        } else if(gamepad2.dpad_down) {
            clawyMcClawClawferson.setLow();
        } else if(gamepad2.dpad_right){
            clawyMcClawClawferson.setMin();
        }

        if(gamepad2.b && !gp2bPressed && !gamepad2.start) {
            if(clawyMcClawClawferson.getStatus() == Claw.Status.CLOSE) {
                clawyMcClawClawferson.open();
            } else {
                clawyMcClawClawferson.close();
            }
        }

        if(gamepad2.a && !gamepad2.start) {
            clawyMcClawClawferson.angleClaw();
        }

        if(gamepad2.x) {
            clawyMcClawClawferson.setEncoderOffset();
        }

        if(gamepad2.y) {
            clawyMcClawClawferson.stop();
        }

        gp2bPressed = gamepad2.b;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        clawyMcClawClawferson.stop();
        drive.stop();
    }

}