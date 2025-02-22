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



@Autonomous(name="Bucket Auto SHORT", preselectTeleOp = "9480 Manual")
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
    double newClawPos = 0.58;


    @Override public void runOpMode()
    {
        clawDrive = hardwareMap.get(Servo.class, "center_claw");
        claw = new Claw(hardwareMap, this);
        slides = new Slides(hardwareMap, this);
        arm = new Arm(hardwareMap, this);
        bot = new Bot(hardwareMap, this);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("this auto will score 11 points max"
        +"putting the preloaded sample in high bucket"
        +"then getting a L1 ascent. ");

        telemetry.addData("claw pos", clawDrive.getPosition());
        telemetry.addData("claw subclass pos", claw.clawDrive.getPosition());

        telemetry.update();
        clawDrive.setPosition(0.58);

        waitForStart();
//        while (yes){
//            slides.HoldLift();
//        }

        clawDrive.setPosition(0.58);
        claw.wristDrive.setPosition(0.36);
        armToBucket();
        sleep(2000);
        driveToClicks(-655, -655, -655,-655, 0.7);
        driveToClicks(-585, -585, 585,585, 0.7);

        driveToClicks(-100, -100, 100, 100, 0.5);
        //slides.setSlidePosition(3000);
        //arm.setArmPosition(Arm.ArmPositions.BUCKET);
        sleep(1000);
        clawDrive.setPosition(0.7);
        sleep(1000);
        claw.wristDrive.setPosition(0.63);
        driveToClicks(300, 300, 300, 300, 0.5);

        sleep(1000);
        armToDrive();

        driveToClicks(1300, 1300, -1300, -1300, 0.5);

        driveToClicks(-2300, -2300, -2300, -2300, 0.5);
        //driving forward
        driveToClicks(985, 985, -985, -985, 0.5);
        //driveToClicks(-2000, -2000, -2000, -2000, 0.5);
        //arm.setArmPosition(Arm.ArmPositions.ASCENT);

        //updateSlideArm();
        sleep(1000);
        armToAscend();
        claw.wristDrive.setPosition(0.4641);
        driveToClicks(-2000, -2000, -2000, -2000, 0.5);
        sleep(1000);
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

    public void armToBucket(){
        slides.setSlidePosition(Slides.SlidePositions.UP);
        arm.setArmPosition(Arm.ArmPositions.BUCKET);
        updateSlideArm();
    }

    public void armToAscend(){
        arm.setArmPosition(Arm.ArmPositions.ASCENT);
        updateSlideArm();
    }

    public void armToDrive(){
        while (!slides.slideLimit.isPressed()) {
            slides.leftSlideDrive.setTargetPosition(slides.leftSlideDrive.getCurrentPosition() - 100);
            slides.rightSlideDrive.setTargetPosition(slides.rightSlideDrive.getCurrentPosition() - 100);

            slides.leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slides.leftSlideDrive.setPower(1);
            slides.rightSlideDrive.setPower(1);

        }
        slides.leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.target = 50;
        arm.setArmPosition(Arm.ArmPositions.HORI);

        updateSlideArm();

    }
    public void updateSlideArm(){
        slides.leftSlideDrive.setTargetPosition(slides.target);
        slides.rightSlideDrive.setTargetPosition(slides.target);
        slides.leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.leftSlideDrive.setPower(0.95);
        slides.rightSlideDrive.setPower(1);

        arm.wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.wormDrive.setTargetPosition(arm.target);
        arm.wormDrive.setPower(1);
    }



}
