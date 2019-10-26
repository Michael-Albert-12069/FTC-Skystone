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
        thumb = hardwareMap.servo.get("thumb");
        finger = hardwareMap.servo.get("finger");
        //declaring inde... names

        waitForStart();
        driveForward(1, 1750);
        turnLeft(1, 2000);
        driveForward(1, 3000);



    }
    public void driveForward(double power, long time) throws InterruptedException {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        Thread.sleep(time);

    }
    public void turnLeft(double power, long time) throws InterruptedException {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
        Thread.sleep(time);
    }
    public void turnRight(double power, long time) throws InterruptedException {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
        Thread.sleep(time);
    }
    public void stopDrive(double power, long time) throws InterruptedException {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        Thread.sleep(time);
    }
}
