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

@TeleOp(name="9480 Manual", group="Linear OpMode")

public class Manual extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
//    public  DcMotor leftFrontDrive = null;
//    public DcMotor leftBackDrive = null;
//    public DcMotor rightFrontDrive = null;
//    public DcMotor rightBackDrive = null;
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
        waitTime = new ElapsedTime();
        claw = new Claw(hardwareMap, this);
        slides = new Slides(hardwareMap, this);
        arm = new Arm(hardwareMap, this);
        bot = new Bot(hardwareMap, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        arm.setArmPosition(Arm.ArmPositions.RESET);
        updateArm();
        arm.wormDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


/*
        arm.wormDrive.setTargetPosition(-1260);
        arm.wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.wormDrive.setPower(1);
        arm.wormDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/

    }


    @Override
    public void loop() {
        bot.moveRobot();

        claw.wristControls();
        arm.HoldArm();
        bot.moveRobot();

        //reset motors
        if (gamepad1.back){
            bot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            bot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (gamepad2.back){
            arm.wormDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slides.leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slides.rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm.wormDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slides.leftSlideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slides.rightSlideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //horizontal limit
        if ((arm.wormDrive.getCurrentPosition() < 830) && slides.rightSlideDrive.getCurrentPosition() > 1550) {
            slides.slideLimitUse();
        }
        else{
            slides.HoldLift();
        }

        //claw positions
        if (gamepad2.right_bumper){
            //claw.toggleClaw();
            claw.setClawPosition(0.7);
        }
        else if(gamepad2.left_bumper){
            claw.setClawPosition(0.58);
        }

        //--------------------------presets-----------------------
        if(gamepad2.dpad_down) slides.setSlidePosition(Slides.SlidePositions.DOWN);
        else if(gamepad2.dpad_left) slides.setSlidePosition(Slides.SlidePositions.MIDDLE);
        else if(gamepad2.dpad_up) slides.setSlidePosition(Slides.SlidePositions.UP);
        else if (gamepad2.dpad_right) slides.setSlidePosition(Slides.SlidePositions.CHAMBER);
        else if(gamepad2.a) {
            slides.setSlidePosition(Slides.SlidePositions.DOWN);
            arm.setArmPosition(Arm.ArmPositions.SAMPLE);
            claw.setWristPosition(0.49);
            claw.openClaw();
        }
        else if(gamepad2.b) {
            slides.setSlidePosition(Slides.SlidePositions.DOWN);
            arm.setArmPosition(Arm.ArmPositions.SPECIMEN);
            claw.setWristPosition(0.48);
            claw.openClaw();
        }
        else if (gamepad2.x){
            claw.setWristPosition(0.41);
        }
        else if(gamepad2.y){
            slides.setSlidePosition(Slides.SlidePositions.CHAMBER);
            arm.setArmPosition(Arm.ArmPositions.CHAMBER);
            claw.setWristPosition(.46);
        }

        telemetry.addData("Left Front", bot.leftFrontDrive.getCurrentPosition());
        telemetry.addData("left back", bot.leftBackDrive.getCurrentPosition());
        telemetry.addData("right front", bot.rightFrontDrive.getCurrentPosition());
        telemetry.addData("right back", bot.rightBackDrive.getCurrentPosition());

        telemetry.addLine()
                        .addData("leftTrigger", gamepad2.left_trigger)
                        .addData("rightTrigger", gamepad2.right_trigger);
        /*telemetry.addData("left front", bot.leftFrontDrive.getCurrentPosition());
        telemetry.addData("right front", bot.rightFrontDrive.getCurrentPosition());
        telemetry.addData("right back", bot.rightBackDrive.getCurrentPosition());
        telemetry.addData("left back", bot.leftBackDrive.getCurrentPosition());*/

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
    public void updateArm(){

        arm.wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.wormDrive.setTargetPosition(arm.target);
        arm.wormDrive.setPower(1);

    }
}
