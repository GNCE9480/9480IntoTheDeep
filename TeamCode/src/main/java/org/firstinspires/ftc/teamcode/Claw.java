package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


public class Claw {
    public Servo clawDrive;
    public Servo wristDrive;

    OpMode lopMode;

    private double clawPosition = 0.7;
    private double wristPosition = 0.7;//change init pos
    private boolean clawOpen = true;

    public Claw(HardwareMap hardwareMap, OpMode opMode){
        clawDrive = hardwareMap.get(Servo.class, "center_claw");
        wristDrive = hardwareMap.get(Servo.class, "center_wrist");
        this.lopMode = opMode;
    }



    public void wristControls(){
        if (Math.abs(lopMode.gamepad2.right_stick_y) > 0.03){
            wristPosition = wristDrive.getPosition() + lopMode.gamepad2.right_stick_y * 0.01;
            if (wristPosition >= 0.7){
                wristPosition = 0.7;
            }
            else if (wristPosition <= 0.3){
                wristPosition = 0.3;
            }
        }
        wristDrive.setPosition(wristPosition);
    }
    public void setWristPosition(double newWristPosition){
        wristPosition = newWristPosition;
    }

    public void toggleClaw(){
        if (clawOpen){
            closeClaw();
        }
        else{
            openClaw();
        }
    }

    public void openClaw(){
        clawPosition = 0.7;

        clawOpen = true;
    }

    public void closeClaw(){
        clawPosition = 0.58;
        clawOpen = false;
    }
    public void setClawPosition(double pos){
        clawPosition = pos;
        clawDrive.setPosition(clawPosition);
        if (pos == 0.7){
            clawOpen = true;
        }
        else if (pos == 0.58){
            clawOpen = false;
        }
    }

}
