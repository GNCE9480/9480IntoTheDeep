package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo clawDrive;
    public Servo wristDrive;

    OpMode lopMode;

    private double clawPosition = 0;
    private double wristPosition = 0.32;

    public Claw(HardwareMap hardwareMap, OpMode opMode){
        clawDrive = hardwareMap.get(Servo.class, "center_claw");
        wristDrive = hardwareMap.get(Servo.class, "center_wrist");
        this.lopMode = opMode;
    }

    public void clawControls(){
        clawDrive.setPosition(clawPosition);
        wristDrive.setPosition(wristPosition);
    }

    public void wristControls(){
        if (Math.abs(lopMode.gamepad2.right_stick_y) > 0.03){
            wristPosition = wristDrive.getPosition() + lopMode.gamepad2.right_stick_y * 0.01;
            if (wristPosition >= 0.5){
                wristPosition = 0.5;
            }
            else if (wristPosition <= 0.3){
                wristPosition = 0.3;
            }
        }
    }

    public void openClaw(){
        clawPosition = 0.7;
    }

    public void closeClaw(){
        clawPosition = 0.58;
    }

    public void setWristPosition(double newWristPosition){
        wristPosition = newWristPosition;
    }
}
