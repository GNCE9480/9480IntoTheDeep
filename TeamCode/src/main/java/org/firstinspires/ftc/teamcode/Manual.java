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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


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
    public  DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor wormDrive = null;
    public DcMotor rightSlideDrive = null;
    public DcMotor leftSlideDrive = null;
    public Servo wristDrive = null;
    public Servo clawDrive = null;
    public TouchSensor slideLimit = null;
    double drive = 0;
    double strafe = 0;
    double turn = 0;
    double arm = 0;
    double wristClicks = 0.32;
    int slideMinInches = 12;
    double clawOpenPos = 0.7;
    double clawClosedPos = 0.58;
    double clawClicks = 0.7;
    ElapsedTime waitTime;



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
        wormDrive = hardwareMap.get(DcMotor.class, "center_arm");
        slideLimit = hardwareMap.get(TouchSensor.class, "armLimitLeft");



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

        armInit();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // helper function to drive
            moveRobot(drive, strafe, turn);
            wormControls();
            wristControls();
            clawControls();
            slideControls();

            telemetry.addData("left front", leftFrontDrive.getCurrentPosition());
            telemetry.addData("right front", rightFrontDrive.getCurrentPosition());
            telemetry.addData("right back", rightBackDrive.getCurrentPosition());
            telemetry.addData("left back", leftBackDrive.getCurrentPosition());
            telemetry.addData("left slide", leftSlideDrive.getCurrentPosition());
            telemetry.addData("right slide", rightSlideDrive.getCurrentPosition());
            telemetry.addData("worm gear", wormDrive.getCurrentPosition());
            telemetry.addData("worm degrees", wormToDeg(wormDrive.getCurrentPosition()));
            telemetry.addData("extension", extendYAxis(wormDrive.getCurrentPosition(), rightSlideDrive.getCurrentPosition()));
            telemetry.addData("slide inches", slideToInches(leftSlideDrive.getCurrentPosition()));
            telemetry.addData("slide is busy", leftSlideDrive.isBusy());
            telemetry.addData("wrist", wristDrive.getPosition());
            telemetry.addData("claw", clawDrive.getPosition());
            telemetry.update();
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
        leftFrontDrive.setPower(leftFrontPower*0.75); //reduce speed here if needed
        rightFrontDrive.setPower(rightFrontPower*0.75);
        leftBackDrive.setPower(leftBackPower*0.75);
        rightBackDrive.setPower(rightBackPower*0.75);

        // Show the wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

        /*
        doesn't work
       if (extendYAxis(wormDrive.getCurrentPosition(), leftSlideDrive.getCurrentPosition()) > 42){
            wormDrive.setPower(0);
            leftSlideDrive.setPower(0);
            rightSlideDrive.setPower(0);
        }
        */

        if(gamepad1.back){
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void slideControls(){
        //manual
        while(gamepad2.left_trigger>0.01) {
            leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftSlideDrive.setPower(-0.6 * gamepad2.left_trigger + 0.2);
            rightSlideDrive.setPower(-0.6 * gamepad2.left_trigger + 0.2);
            if (slideLimit.isPressed()){
                leftSlideDrive.setPower(0);
                rightSlideDrive.setPower(0);
                leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }

        while(gamepad2.right_trigger>0.01) {
            leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftSlideDrive.setPower(0.6 * gamepad2.right_trigger);
            rightSlideDrive.setPower(0.6 * gamepad2.right_trigger);
            if (slideLimit.isPressed()){
                leftSlideDrive.setPower(0);
                rightSlideDrive.setPower(0);
                leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }

        if (slideLimit.isPressed()) {
            leftSlideDrive.setPower(0);
            rightSlideDrive.setPower(0);
            leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //presets
        if(gamepad2.dpad_down){
            moveSlides(0, 0.5);
        }
        if(gamepad2.dpad_left){
            moveSlides(-1000, 0.5);
        }
        if(gamepad2.dpad_up){
            moveSlides(-2000, 0.5);
        }

        leftSlideDrive.setTargetPosition(leftSlideDrive.getCurrentPosition());
        rightSlideDrive.setTargetPosition(rightSlideDrive.getCurrentPosition());

        /*
        picking up sample from submersible
        if (gamepad2.a){
            moveArm()
        }
         */

        //getting specimen
        if (gamepad2.b){
            moveArm(0, 0.5);
            moveWrist(0.44);
            moveSlides(0, 0.5);
            openClaw();
        }

        leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //horizontal limit
        if (leftSlideDrive.getCurrentPosition()<-1550 && wormDrive.getCurrentPosition() < 830){
            leftSlideDrive.setPower(0);
            rightSlideDrive.setPower(0);
        }

/*
redundant or do we need?
        if (slideLimit.isPressed()) {
            leftSlideDrive.setPower(0);
            rightSlideDrive.setPower(0);
        }

 */

        if (gamepad2.back){
            leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wormDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wormDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void wormControls(){
        wormDrive.setPower(-0.6 * gamepad2.left_stick_y);
        //limit
        if (wormDrive.getCurrentPosition()>2300) {
            wormDrive.setPower(0);
        }
    }
    public void clawControls(){
        if (gamepad2.right_bumper){
            openClaw();
        }
        else if (gamepad2.left_bumper) {
            closeClaw();
        }
    }

    public void openClaw(){
        clawClicks = clawOpenPos;
        clawDrive.setPosition(clawClicks);
    }
    public void closeClaw(){
        clawClicks = clawClosedPos;
        clawDrive.setPosition(clawClicks);
    }

    public void wristControls(){
        if (Math.abs(gamepad2.right_stick_y) > 0.03){
            wristClicks = wristDrive.getPosition()+gamepad2.right_stick_y*0.01;//0.05 is just the turn rate
            if (wristClicks >= 0.5){ //0.05 is placeholder for wrist limit
                wristClicks = 0.5;
            }
            else if (wristClicks <= 0.3){
                wristClicks = 0.3;
            }
            wristDrive.setPosition(wristClicks);
        }
        telemetry.addData("Wrist Clicks", wristDrive.getPosition());
    }
    public void moveArm(int wormPos, double wormPow){
        waitTime = new ElapsedTime();

        wormDrive.setTargetPosition(wormPos);
        wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wormDrive.setPower(wormPow);

        while (wormDrive.isBusy() && opModeInInit() && waitTime.seconds() < 4) {

        }
        wormDrive.setPower(0);
        wormDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void moveSlides(int Pos, double power) {
        waitTime = new ElapsedTime();

        leftSlideDrive.setPower(power);
        rightSlideDrive.setPower(power);
        //leftSlideDrive.setTargetPosition(Pos);
        //rightSlideDrive.setTargetPosition(Pos);

        /*
        leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */

        while ((leftSlideDrive.isBusy() && rightSlideDrive.isBusy()) && opModeInInit() && waitTime.seconds() < 4) {
        }
        while (leftSlideDrive.getCurrentPosition() != Pos && rightSlideDrive.getCurrentPosition() != Pos && opModeIsActive()){
            leftSlideDrive.setTargetPosition(Pos);
            rightSlideDrive.setTargetPosition(Pos);
        }
    }

    public void moveWrist(double Pos){
        wristDrive.setPosition(Pos);
    }
    public void armInit(){
        wristDrive.setPosition(0.32);
        clawDrive.setPosition(clawClosedPos);

        waitTime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        if (!slideLimit.isPressed()){
            leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlideDrive.setPower(-0.3);
            rightSlideDrive.setPower(-0.3);

            leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlideDrive.setPower(0);
            rightSlideDrive.setPower(0);
        }
        else {
            leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        leftSlideDrive.setPower(0);
        rightSlideDrive.setPower(0);
        leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlideDrive.setTargetPosition(0);
        rightSlideDrive.setTargetPosition(0);
        while (!slideLimit.isPressed()){
            leftSlideDrive.setPower(0.5);
            rightSlideDrive.setPower(0.5);
        }

        //reset shoulder
        wormDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    private int slideToClicks(double inch){
        return (int) ((inch - slideMinInches) * -84.7);
    }
    private double slideToInches(double clicks) {
        return clicks / 84.7 + slideMinInches;
    }
    public int wormToDeg(double wormPosClicks){
        int output = (int)Math.round(0.036*wormPosClicks);
        return output; // - make sure this works/ that the conversion rate is correct
    }

    //doesn't work
    private double extendYAxis(double wormClicks, double slideClicks){
        double slideInch = slideClicks / 84.7 + slideMinInches;
        double wormDeg = (-401 * wormClicks +200)+0;
        return Math.cos(Math.toRadians(wormDeg))*slideInch;
    }



}
