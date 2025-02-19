package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.CoreOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Arm;



@Autonomous(name="Red Bucket Auto")
public class RedBucket extends LinearOpMode {
    //Manual man = new Manual();

    private ElapsedTime runtime = new ElapsedTime();

    ElapsedTime waitTime;
    Bot bot;
    Arm arm;
    Slides slides;
    Claw claw;
    public Servo clawDrive;
    double clawOpen = 0.7;
    double clawClose = 0.58;
    boolean yes = true;


    @Override public void runOpMode()
    {
        clawDrive = hardwareMap.get(Servo.class, "center_claw");
        claw = new Claw(hardwareMap, this);
        slides = new Slides(hardwareMap, this);
        arm = new Arm(hardwareMap, this);
        bot = new Bot(hardwareMap, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        moveArm(1900, 0.5, 3800, 0.5, 0.45);
        driveToClicks(-650, -650, -650,-650, 0.7);
        driveToClicks(-600, -600, 600,600, 0.7);
        slides.setSlidePosition(Slides.SlidePositions.UP);
        arm.setArmPosition(Arm.ArmPositions.BUCKET);

        driveToClicks(370, -370, -370, 370, 0.7);

        clawDrive.setPosition(0.58);


    }

    private void stopAndResetMotors(){
        bot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       bot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void driveToClicks(int clicksleftFrontDrive, int clicksleftBackDrive, int clicksrightFrontDrive, int clicksrightBackDrive, double power){
        stopAndResetMotors();
        bot.leftFrontDrive.setTargetPosition(clicksleftFrontDrive);
        bot.leftBackDrive.setTargetPosition(clicksleftBackDrive);
        bot.rightFrontDrive.setTargetPosition(clicksrightFrontDrive);
        bot.rightBackDrive.setTargetPosition(clicksrightBackDrive);

        bot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bot.leftFrontDrive.setPower(power);
        bot.leftBackDrive.setPower(power);
        bot.rightFrontDrive.setPower(power);
        bot.rightBackDrive.setPower(power);
        while( bot.leftBackDrive.isBusy()&& bot.leftFrontDrive.isBusy()&&bot.rightBackDrive.isBusy()&&bot.rightFrontDrive.isBusy()){

        }

    }
    public void moveArm(int wormPos, double wormPow, int slidesPos, double slidePow,  double wristPos){

        arm.wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.wormDrive.setTargetPosition(wormPos);
        arm.wormDrive.setPower(wormPow);

        slides.leftSlideDrive.setTargetPosition(slidesPos);
        slides.rightSlideDrive.setTargetPosition(slidesPos);

        slides.leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides.leftSlideDrive.setPower(slidePow);
        slides.rightSlideDrive.setPower(slidePow);


        while (arm.wormDrive.isBusy()  &&  slides.leftSlideDrive.isBusy()  &&  slides.rightSlideDrive.isBusy()){

        }
        claw.setWristPosition(wristPos);




    }


}
