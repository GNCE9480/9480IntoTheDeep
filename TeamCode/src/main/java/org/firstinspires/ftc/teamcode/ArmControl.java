package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        wormGear = myOpMode.hardwareMap.get(DcMotor.class, "worm_gear");
        leftSlide = myOpMode.hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = myOpMode.hardwareMap.get(DcMotor.class, "right_slide");
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");

    }

}
