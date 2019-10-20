package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Arm Test",group = "TeleOp")
public class ArmTest extends LinearOpMode {
    private DcMotor armMotor;
    @Override
    public void runOpMode() throws InterruptedException{
        //this where Op code is owo
        armMotor= hardwareMap.dcMotor.get("arm");



        waitForStart();
        //below is what happens when you press start
        while (opModeIsActive()){
            armMotor.setPower(+gamepad1.right_trigger);
            armMotor.setPower(-gamepad1.right_trigger);
        }

        telemetry.update();
        idle();
    }
}
