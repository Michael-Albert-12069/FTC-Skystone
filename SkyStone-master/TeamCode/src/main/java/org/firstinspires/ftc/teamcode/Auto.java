package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "autonomous test", group = "Autonomous")
public class Auto extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo thumb;
    private Servo finger;
    private int targetposition;


    @Override
    public void runOpMode() throws InterruptedException{
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
    //    thumb = hardwareMap.servo.get("thumb");
    //    finger = hardwareMap.servo.get("finger");
//insert Ryer slam head on table

        waitForStart();
        driveForward(40);
        turnLeft(115);
        driveForward(106);






    }
    public void driveForward(int cm) throws InterruptedException {
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        int rightPos = rightMotor.getCurrentPosition();
        int leftPos = leftMotor.getCurrentPosition();

        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        rightMotor.setTargetPosition(rightPos + (10*cm));
        leftMotor.setTargetPosition(leftPos+ (10*cm));
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(1);
        leftMotor.setPower(1);
        while (leftMotor.isBusy() && rightMotor.isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void turnLeft(int degrees) throws InterruptedException {
        int cm =  degToCM(degrees);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        int rightPos = rightMotor.getCurrentPosition();
        int leftPos = leftMotor.getCurrentPosition();

        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        rightMotor.setTargetPosition(rightPos - (10*cm));
        leftMotor.setTargetPosition(leftPos+ (10*cm));
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(-1);
        leftMotor.setPower(1);
        while (leftMotor.isBusy() && rightMotor.isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void turnRight(int degrees) throws InterruptedException {
        int cm =  degToCM(degrees);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        int rightPos = rightMotor.getCurrentPosition();
        int leftPos = leftMotor.getCurrentPosition();
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        rightMotor.setTargetPosition(rightPos + (10*cm));
        leftMotor.setTargetPosition(leftPos - (10*cm));
        rightMotor.setPower(1);
        leftMotor.setPower(-1);
        while (leftMotor.isBusy() && rightMotor.isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void stopDrive() throws InterruptedException {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public static int degToCM(int deg){
        int cm = (int) Math.sqrt(120*deg);
        cm-=(0.2*deg);
        return cm;
    }
}
