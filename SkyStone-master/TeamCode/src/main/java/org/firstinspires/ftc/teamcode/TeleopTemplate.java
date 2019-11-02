package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Base Teleop",group = "TeleOp")
public class TeleopTemplate extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo thumb;
    private Servo finger;
    @Override
    public void runOpMode() throws InterruptedException{
        //this where Op code is owo
        leftMotor= hardwareMap.dcMotor.get("left");
        rightMotor= hardwareMap.dcMotor.get("right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        thumb = hardwareMap.servo.get("thumb");
        finger = hardwareMap.servo.get("finger");



        waitForStart();
        //below is what happens when you press start
        while (opModeIsActive()){
            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.right_stick_y);
        }

        telemetry.update();
        idle();
    }
}
