package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Slides {
    public DcMotorEx leftSlideDrive;
    public DcMotorEx rightSlideDrive;
    public TouchSensor slideLimit;

    OpMode lopMode;

    int target = 0;

    public Slides(HardwareMap hardwareMap, OpMode opMode){
        leftSlideDrive = hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlideDrive = hardwareMap.get(DcMotorEx.class, "right_slide");
        slideLimit = hardwareMap.get(TouchSensor.class, "armLimitLeft");

        leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlideDrive.setTargetPosition(target);
        rightSlideDrive.setTargetPosition(target);


        leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        this.lopMode = opMode;
    }

    public void HoldLift(){
        if(slideLimit.isPressed()){
            leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            target = 50;
        }



        if(Math.abs(lopMode.gamepad2.left_trigger) > 0.1 || Math.abs(lopMode.gamepad2.right_trigger) > 0.1){
            final double sensitivity = 0.8;
            leftSlideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            final double power = sensitivity * (lopMode.gamepad2.left_trigger - lopMode.gamepad2.right_trigger);
            leftSlideDrive.setPower(power);
            rightSlideDrive.setPower(power);

            target = rightSlideDrive.getCurrentPosition();
        } else {
            leftSlideDrive.setTargetPosition(target);
            rightSlideDrive.setTargetPosition(target);
            leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideDrive.setPower(1);
            rightSlideDrive.setPower(1);
        }
    }

    public enum SlidePositions {
        DOWN,
        MIDDLE,
        UP,
        CHAMBER,
        MD, //middle down
    }

    public void setSlidePosition(SlidePositions targetSlidePosition){
        switch(targetSlidePosition){
            case UP:
                target = 2800;
                break;
            case MIDDLE:
                target = 1000;
                break;
            case DOWN:
                target = 0;
                break;
            case CHAMBER:
                target = 796;
            case MD:
                target = 500;
            default:
                break;
        }
    }

    public void slideLimitUse(){
        rightSlideDrive.setTargetPosition(1550);
        leftSlideDrive.setTargetPosition(1550);
        rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideDrive.setPower(1);
        leftSlideDrive.setPower(1);
        }

    public int getTarget(){
        return target;
    }
}
