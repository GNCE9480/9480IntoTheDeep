package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public DcMotorEx wormDrive;
    OpMode lopMode;
    private int target = 0;
    public Arm(HardwareMap hardwareMap, OpMode opMode){
        wormDrive = hardwareMap.get(DcMotorEx.class, "center_arm");
        wormDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        wormDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wormDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormDrive.setTargetPosition(target);
        wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lopMode = opMode;
    }
    public void HoldArm(){
        final int UpperLimit = 2500;
        final int LowerLimit = -100;

        if(lopMode.gamepad2.left_stick_y != 0){
            if(wormDrive.getCurrentPosition() > UpperLimit) {
                wormDrive.setTargetPosition(UpperLimit);
                wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormDrive.setPower(1);
            } else if (wormDrive.getCurrentPosition() < LowerLimit){
                wormDrive.setTargetPosition(LowerLimit);
                wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormDrive.setPower(1);
            } else {
                final double sensitivity = 0.8;
                wormDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                final double power = sensitivity * (lopMode.gamepad2.left_stick_y);
                wormDrive.setPower(power);

                target = wormDrive.getCurrentPosition();
            }

        } else {
            if(target > 2500) target = 2500;
            else if(target < 0) target = 0;
            wormDrive.setTargetPosition(target);
            wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wormDrive.setPower(0.7);
        }
    }

    public enum ArmPositions {
        SAMPLE,
        SPECIMEN,
        CHAMBER,
        BUCKET,
    }

    public void setArmPosition(ArmPositions targetSlidePosition){
        switch(targetSlidePosition){
            case SAMPLE:
                target = 1;
                break;
            case SPECIMEN:
                target = 192;
                break;
            case CHAMBER:
                target = 968;
                break;
            case BUCKET:
                target = 1940;
                break;
            default:
                break;
        }
    }

    public int getTarget(){
        return target;
    }
}
