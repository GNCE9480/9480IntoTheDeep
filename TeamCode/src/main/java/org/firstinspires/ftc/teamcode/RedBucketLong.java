package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.CoreOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Bucket Auto LONG")
@Disabled
public class RedBucketLong extends LinearOpMode {
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
        telemetry.addLine("this auto will score 19 points max"
        +" putting the preloaded sample in high bucket"
        +" then getting one more sample then getting a L1 ascent. ");

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
        driveToClicks(-650, -650, -650,-650, 0.7);
        driveToClicks(-550, -550, 550,550, 0.7);
        //slides.setSlidePosition(3000);
        //arm.setArmPosition(Arm.ArmPositions.BUCKET);
        sleep(1000);
        clawDrive.setPosition(0.7);
        sleep(1000);
        claw.wristDrive.setPosition(0.63);
        driveToClicks(300, 300, 300, 300, 0.5);

        sleep(1000);
        armToDrive();

        driveToClicks(1250, 1250, -1250, -1250, 0.5);

//        driveToClicks(370, -370, -370, 370, 0.5);
//
//        driveToClicks(1310, 1465, -1310, -1465, 0.5);
//        driveToClicks(-900, -900, -900, -900, 0.5);
//         armToDrive();
//        sleep(1000);
//        claw.wristDrive.setPosition(0.4105);
//        clawDrive.setPosition(0.58);


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



        slides.leftSlideDrive.setTargetPosition(slidesPos);
        slides.rightSlideDrive.setTargetPosition(slidesPos);

        slides.leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides.leftSlideDrive.setPower(slidePow);
        slides.rightSlideDrive.setPower(slidePow);

        slides.leftSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.rightSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (arm.wormDrive.isBusy()  &&  slides.leftSlideDrive.isBusy()  &&  slides.rightSlideDrive.isBusy()){

        }
        claw.setWristPosition(wristPos);
    }
    public void armToBucket(){
        slides.setSlidePosition(Slides.SlidePositions.CHAMBER);
        arm.setArmPosition(Arm.ArmPositions.BUCKET);
        updateSlideArm();


    }

    public void armToDrive(){
        while (!slides.slideLimit.isPressed()){
            slides.leftSlideDrive.setTargetPosition(slides.leftSlideDrive.getCurrentPosition()-100);
            slides.rightSlideDrive.setTargetPosition(slides.rightSlideDrive.getCurrentPosition()-100);

            slides.leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slides.leftSlideDrive.setPower(1);
            slides.rightSlideDrive.setPower(1);

        }
        slides.leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.target = 50;
        arm.setArmPosition(Arm.ArmPositions.SAMPLE);
        /*slides.leftSlideDrive.setTargetPosition(0);
        slides.rightSlideDrive.setTargetPosition(0);

        slides.leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides.leftSlideDrive.setPower(1);
        slides.rightSlideDrive.setPower(1);*/

        //slides.setSlidePosition(Slides.SlidePositions.DOWN);
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
