package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//Voltron Code
//Do do do do do do
//DO DO
//Dooo Doooo DOoooooo

@TeleOp(name = "Base Teleop",group = "TeleOp")
public class TeleOpComp1 extends LinearOpMode{
    private DcMotor armMotor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo rAS;
    private Servo lAS;
    private Servo rCS;
    private Servo lCS;
    private Servo thumb;
    private Servo finger;
    double thumbAngle = 0.0;


    //ahhhhhh it fast forward travel
    @Override

    public void runOpMode() throws InterruptedException{
        //this where Op code is owo
        armMotor= hardwareMap.get(DcMotor.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rAS = hardwareMap.servo.get("rightAS");
        lAS = hardwareMap.servo.get("leftAS");
        rCS = hardwareMap.servo.get("rightCS");
        lCS = hardwareMap.servo.get("leftCS");

//and then you Knowles what happened

        double initArmPosL = lCS.getPosition();
        double initArmPosR = rCS.getPosition();

        //this where Op code is owo
        leftMotor= hardwareMap.dcMotor.get("left");
        rightMotor= hardwareMap.dcMotor.get("right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        thumb = hardwareMap.servo.get("thumb");
        finger = hardwareMap.servo.get("finger");
//LVP goes to Ryer
//Ryer: Thank you, this is an honor

        waitForStart();

        //below is what happens when you press start
        while (opModeIsActive()){
            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.right_stick_y);
        }

        telemetry.update();
        idle();

        //below is what happens when you press start
        while (opModeIsActive()){
            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.right_stick_y);
        }

        telemetry.update();
        idle();
        //below is what happens when you press start
        while (opModeIsActive()){
            double rPotentialPos = initArmPosR+(gamepad2.right_trigger/10)-(gamepad2.left_trigger/10);
            double lPotentialPos = initArmPosL-(gamepad2.right_trigger/10)+(gamepad2.left_trigger/10);
            lCS.setPosition(lPotentialPos);
            rCS.setPosition(rPotentialPos);
            //+ and - could be flipped
            int lASopenPosition = 0;
            int lASclosePosition = 0;
            int rASclosePosition = 0;
            int rASopenPosition = 0;
            if(gamepad2.a){
                lAS.setPosition(lASopenPosition);
                rAS.setPosition(rASopenPosition);
            }
            if(gamepad2.b) {
                lAS.setPosition(lASclosePosition);
                rAS.setPosition(rASclosePosition);
            }
            //sets the claw to open on A and close on B
            armMotor.setPower(gamepad2.left_stick_y*1);




//Alto Clef makes more sense than Treble




            telemetry.addLine("Arm Position: " + armMotor.getCurrentPosition());




            telemetry.update();
        }



        idle();
    }
    public static int posToTicks(int position){
        if (position == 1){
            return 0;
        }else if(position == 2){
            return -45;
        }else if(position ==3){
            return -70;
        }else if(position ==4){
            return -105;
        }else {
            return -45;

        }
    }
}