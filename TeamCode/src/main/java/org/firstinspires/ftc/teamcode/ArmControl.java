package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmControl {
    private LinearOpMode myOpMode = null;
    private DcMotor wormGear = null;
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private Servo claw = null;
    private Servo wrist = null;
    //put all variables, methods, driver inputs and other for arms here
    //REMEMBER ARM INIT
    public void armInit() {


        wormGear.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);


    }

    public void setArm(double wormGear, double slides, double claw, double wrist){

    }


}
