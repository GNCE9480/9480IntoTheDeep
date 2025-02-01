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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="9480 Manual", group="Linear OpMode")

public class Manual extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //public  DcMotor leftFrontDrive = null;
    //public DcMotor leftBackDrive = null;
    //public DcMotor rightFrontDrive = null;
    //public DcMotor rightBackDrive = null;
    double drive = 0;
    double strafe = 0;
    double turn = 0;
    int slideMinInches = 12;
    ElapsedTime waitTime;

    Slides slides;
    Claw claw;
    Arm arm;
    Bot bot;


    @Override
    public void init(){
        /*
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
         */

        claw = new Claw(hardwareMap, this);
        slides = new Slides(hardwareMap, this);
        arm = new Arm(hardwareMap, this);
        bot = new Bot(hardwareMap, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        bot.moveRobot();
        slides.HoldLift();
        claw.clawControls();
        claw.wristControls();
        arm.HoldArm();

        if (gamepad2.back) bot.reset();
        if (gamepad2.right_bumper) {
            claw.toggleClaw();
        }
        //horizontal limit
        if (arm.wormDrive.getCurrentPosition() > 830) {
            slides.slideLimit(1550);
        }
        //--------------------------presets-----------------------
        if(gamepad2.dpad_down) slides.setSlidePosition(Slides.SlidePositions.DOWN);
        else if(gamepad2.dpad_left) slides.setSlidePosition(Slides.SlidePositions.MIDDLE);
        else if(gamepad2.dpad_up) slides.setSlidePosition(Slides.SlidePositions.UP);
        else if(gamepad2.a) {
            slides.setSlidePosition(Slides.SlidePositions.DOWN);
            arm.setArmPosition(Arm.ArmPositions.SAMPLE);
        }
        else if(gamepad2.b) {
            slides.setSlidePosition(Slides.SlidePositions.DOWN);
            arm.setArmPosition(Arm.ArmPositions.SPECIMEN);
            claw.setWristPosition(0.43);
            claw.openClaw();
        }
        else if(gamepad2.y){
            slides.setSlidePosition(Slides.SlidePositions.CHAMBER);
            arm.setArmPosition(Arm.ArmPositions.CHAMBER);
        }

        telemetry.addLine()
                        .addData("leftTrigger", gamepad2.left_trigger)
                        .addData("rightTrigger", gamepad2.right_trigger);
        telemetry.addData("left front", bot.leftFrontDrive.getCurrentPosition());
        telemetry.addData("right front", bot.rightFrontDrive.getCurrentPosition());
        telemetry.addData("right back", bot.rightBackDrive.getCurrentPosition());
        telemetry.addData("left back", bot.leftBackDrive.getCurrentPosition());

        telemetry.addData("current slide target", slides.getTarget());
        telemetry.addData("arm target", arm.getTarget());
        //telemetry.addData("Front left/Right", "%4.2f, %4.2f", bot.leftFrontPower, bot.rightFrontPower);
        //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", bot.leftBackPower, bot.rightBackPower);

        telemetry.addData("worm degrees", wormToDeg(arm.wormDrive.getCurrentPosition()));
        telemetry.addData("extension", extendYAxis(arm.wormDrive.getCurrentPosition(), slides.rightSlideDrive.getCurrentPosition()));
        telemetry.addData("slide inches", slideToInches(slides.leftSlideDrive.getCurrentPosition()));

        telemetry.addData("left slide", slides.leftSlideDrive.getCurrentPosition());
        telemetry.addData("right slide", slides.rightSlideDrive.getCurrentPosition());
        telemetry.addData("worm gear", arm.wormDrive.getCurrentPosition());
        telemetry.addData("wrist", claw.wristDrive.getPosition());
        telemetry.addData("claw", claw.clawDrive.getPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
    /*
    public void moveRobot(){
        double axial    =  gamepad1.left_stick_y;
        double lateral  =  gamepad1.left_stick_x;
        double yaw      =  gamepad1.right_stick_x;

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
     */

    //----------------------------conversions--------------------------
    private int slideToClicks(double inch){
        return (int) ((inch - slideMinInches) * -84.7);
    }
    private double slideToInches(double clicks) {
        return clicks / 84.7 + slideMinInches;
    }
    public int wormToDeg(double wormPosClicks){
        return (int)Math.round(0.036*wormPosClicks); // - make sure this works/that the conversion rate is correct
    }
    public int degToWorm(double deg){
        return (int)(Math.round(deg/0.036));
    }
    private double extendYAxis(double wormClicks, double slideClicks){
        double slideInch = slideClicks / 84.7 + slideMinInches;
        double wormDeg = (-401 * wormClicks +200)+0;
        return Math.cos(Math.toRadians(wormDeg))*slideInch;
    }
}
