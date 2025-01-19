/* Copyright (c) 2021 FIRST. All rights reserved.
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

import java.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;


import java.util.List;
import java.util.concurrent.TimeUnit;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="9480 Manual", group="Linear OpMode") //change name "" to show different name in the app interface

public class Manual extends LinearOpMode {
    //ArmControl armCenter = new ArmControl(); //object used to call methods from within our arms class.
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armDrive = null;
    private DcMotor rightSlideDrive = null;
    private DcMotor leftSlideDrive = null;
    private Servo wristDrive = null;
    private Servo clawDrive = null;
    double drive = 0;
    double strafe = 0;
    double turn = 0;
    double arm = 0;
    double slides = 0;
    double wrist = 0;
    double claw = 0;
    double wristClicks = 0; //TODO - set figure this out on team computer
    double slideMinInches = 11.81;



    @Override

    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        //Open the FTC Robot controller app on the phone, turn on the robot, and pair the two.
        // find your way to the config panel for the control hub. make sure the motors are paired with the right plugs
        // name them appropriately(left_front, left_back, etc). make names match the 'deviceName' tab here.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        clawDrive = hardwareMap.get(Servo.class, "center_claw");
        wristDrive = hardwareMap.get(Servo.class, "center_wrist");
        leftSlideDrive = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlideDrive = hardwareMap.get(DcMotor.class, "right_slide");
        armDrive = hardwareMap.get(DcMotor.class, "center_arm");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        //set to personal preference
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftSlideDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlideDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // helper function to drive
            moveRobot(drive, strafe, turn);
            moveArm(arm, slides, wrist, claw);
            wristControls();

            telemetry.addData("left front", leftFrontDrive.getCurrentPosition());
            telemetry.addData("right front", rightFrontDrive.getCurrentPosition());
            telemetry.addData("right back", rightBackDrive.getCurrentPosition());
            telemetry.addData("left back", leftBackDrive.getCurrentPosition());
            telemetry.addData("slides", leftSlideDrive.getCurrentPosition());
            telemetry.addData("arm", armDrive.getCurrentPosition());
            telemetry.addData("wrist", wristDrive.getPosition());
            telemetry.addData("claw", clawDrive.getPosition());
            telemetry.update();
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.


            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

/*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
*/

        }

    }
    public void moveRobot(double axial, double lateral, double yaw){
        axial    =  gamepad1.left_stick_y;
        lateral  =  gamepad1.left_stick_x;
        yaw      =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower); //reduce speed here if needed
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Show the wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        if (extendYAxis(armDrive.getCurrentPosition(), leftSlideDrive.getCurrentPosition()) > 42){
            armDrive.setPower(0);
            leftSlideDrive.setPower(0);
            rightSlideDrive.setPower(0);
        }
    }

    public void moveArm(double arm, double slides, double wrist, double claw){
        arm    =  gamepad2.left_stick_x;
        slides = gamepad2.left_stick_y;
        wrist = gamepad2.right_stick_y;
        claw = gamepad2.right_stick_x;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double armPower  = arm * 0.9;
        double slidesPower = slides * 0.75;
        double wristPower   = wrist * 1.0;
        double clawPower  = claw * 1.0;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(armPower), Math.abs(slidesPower));
        max = Math.max(max, Math.abs(wristPower));
        max = Math.max(max, Math.abs(clawPower));

        if (max > 1.0) {
            armPower  /= max;
            slidesPower /= max;
            wristPower   /= max;
            clawPower  /= max;
        }
        // Send calculated power to wheels
        armDrive.setPower(armPower);
        leftSlideDrive.setPower(slidesPower);
        rightSlideDrive.setPower(slidesPower);
        wristDrive.setPosition(wristPower);
        clawDrive.setPosition(clawPower);
        // Show the arm power.
        telemetry.addData("Slides", "%4.2f", slidesPower);
        telemetry.addData("Arm", "%4.2f", armPower);
        telemetry.addData("Slides", "%4.2f", wristPower);
        telemetry.addData("Arm", "%4.2f", clawPower);



    }

    public void clawControls(){


    }
    public void wristControls(){
        if (gamepad2.right_stick_y > 0.01){
            wristClicks = wristDrive.getPosition()+gamepad2.right_stick_y*0.05;//0.05 is just the turn rate
            if (wristClicks >= 0.05){ //0.05 is placeholder for wrist limit
                wristClicks = 0.05;
            }
            else if (wristClicks <= 0){
                wristClicks = 0;
            }
            wristDrive.setPosition(wristClicks);
        }
        telemetry.addData("Wrist Clicks", wristDrive.getPosition());
    }
    public double wormToDeg(double wormPosClicks){
        return (-401 * wormPosClicks +200)+0; //TODO - make sure this works/ that the conversion rate is correct
    }
    private double slideToInches(double clicks) {
        return clicks / 84.7 + slideMinInches;
    }
    private double extendYAxis(double wormClicks, double slideClicks){
        double slideInch = slideClicks / 84.7 + slideMinInches;
        double wormDeg = (-401 * wormClicks +200)+0;
        return Math.cos(Math.toRadians(wormDeg))*slideInch;

    }

}
