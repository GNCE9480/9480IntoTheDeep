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



    @Override public void runOpMode()
    {
        man.armInit();



        man.moveArm(100, 0.5);

        driveToClicks(100,100,100,100, 0.5);



    }

    private void stopAndResetMotors(){
        man.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        man.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        man.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        man.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void driveToClicks(int clicksleftFrontDrive, int clicksleftBackDrive, int clicksrightFrontDrive, int clicksrightBackDrive, double power){
        stopAndResetMotors();
        man.leftFrontDrive.setTargetPosition(clicksleftFrontDrive);
        man.leftBackDrive.setTargetPosition(clicksleftBackDrive);
        man.rightFrontDrive.setTargetPosition(clicksrightFrontDrive);
        man.rightBackDrive.setTargetPosition(clicksrightBackDrive);

        man.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        man.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        man.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        man.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        man.leftFrontDrive.setPower(power);
        man.leftBackDrive.setPower(power);
        man.rightFrontDrive.setPower(power);
        man.rightBackDrive.setPower(power);
        while ((man.leftBackDrive.isBusy() || man.rightFrontDrive.isBusy())) {

        }
    }

}
