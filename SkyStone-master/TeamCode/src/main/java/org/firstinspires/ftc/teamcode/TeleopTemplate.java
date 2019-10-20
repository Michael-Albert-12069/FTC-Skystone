package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Teleop Template",group = "TeleOp")
public class TeleopTemplate extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    @Override
    public void runOpMode() throws InterruptedException{
        //this where Op code is owo
        leftMotor= hardwareMap.dcMotor.get("left");
        rightMotor= hardwareMap.dcMotor.get("right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();
        //below is what happens when you press start
        while (opModeIsActive()){
            leftMotor.setPower(gamepad1.left_stick_y/2);
            rightMotor.setPower(gamepad1.right_stick_y/2);
        }

        telemetry.update();
        idle();
    }
}
