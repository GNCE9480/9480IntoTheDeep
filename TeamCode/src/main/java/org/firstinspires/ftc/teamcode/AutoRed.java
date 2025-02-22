package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.CoreOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@Autonomous(name="AutoRedClose", preselectTeleOp = "9480 Manual")
public class AutoRed extends LinearOpMode {
    Manual man = new Manual();

    private ElapsedTime runtime = new ElapsedTime();
    public  DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor wormDrive = null;
    public DcMotor rightSlideDrive = null;
    public DcMotor leftSlideDrive = null;
    public Servo wristDrive = null;
    public Servo clawDrive = null;
    public TouchSensor slideLimit = null;
    ElapsedTime waitTime;




    @Override public void runOpMode()
    {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        clawDrive = hardwareMap.get(Servo.class, "center_claw");
        wristDrive = hardwareMap.get(Servo.class, "center_wrist");
        leftSlideDrive = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlideDrive = hardwareMap.get(DcMotor.class, "right_slide");
        wormDrive = hardwareMap.get(DcMotor.class, "center_arm");
        slideLimit = hardwareMap.get(TouchSensor.class, "armLimitLeft");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //man.armInit();
        waitForStart();



        moveArm(100, 0.5);

        driveToClicks(1000,1000,1000,1000, 1);





    }

    private void stopAndResetMotors(){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void driveToClicks(int clicksleftFrontDrive, int clicksleftBackDrive, int clicksrightFrontDrive, int clicksrightBackDrive, double power){
        stopAndResetMotors();
        leftFrontDrive.setTargetPosition(clicksleftFrontDrive);
        leftBackDrive.setTargetPosition(clicksleftBackDrive);
        rightFrontDrive.setTargetPosition(clicksrightFrontDrive);
        rightBackDrive.setTargetPosition(clicksrightBackDrive);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        while( leftBackDrive.isBusy()&& leftFrontDrive.isBusy()&&rightBackDrive.isBusy()&&rightFrontDrive.isBusy()){

        }

    }
    public void moveArm(int wormPos, double wormPow){
        waitTime = new ElapsedTime();

        wormDrive.setTargetPosition(wormPos);
        wormDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wormDrive.setPower(wormPow);

        while (wormDrive.isBusy() && opModeInInit() && waitTime.seconds() < 4) {

        }
        wormDrive.setPower(0);
        wormDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


}
