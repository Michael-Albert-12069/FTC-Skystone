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

// /n
//

@TeleOp(name = "Competition TeleOp",group = "TeleOp")
public class TeleOpComp1 extends LinearOpMode{
    private DcMotor armMotor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo rAS;
    private Servo lAS;
    private Servo rCS;
    private Servo lCS;
    double lASclosePosition = 0.9;
    double lASopenPosition = 0.5;
    double rASclosePosition = 0.6;
    double rASopenPosition = 0.3;
    int armPosition;



    @Override
    public void runOpMode(){
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rAS = hardwareMap.servo.get("rightAS");
        rAS.setDirection(Servo.Direction.REVERSE);

        lAS = hardwareMap.servo.get("leftAS");
        rCS = hardwareMap.servo.get("rightCS");

        lCS = hardwareMap.servo.get("leftCS");
        lCS.setDirection(Servo.Direction.FORWARD);



        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

int position = 0;

        waitForStart();

        //below is what happens when you press start
        while (opModeIsActive()) {
            if (gamepad2.dpad_up){
                if (position <= 4){
                    position += 1;
                }


            }

            if (gamepad2.dpad_down){
                if (position >= 0){
                    position -= 1;
                }
            }

            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.right_stick_y);





            if (gamepad2.a) {//open
                lAS.setPosition(0.5);
                rAS.setPosition(0.3);
            }
            if (gamepad2.b) {//close
                lAS.setPosition(0.9);
                rAS.setPosition(0.6);
            }
            //sets the claw to open on A and close on B

         //   armMotor.setPower(gamepad2.right_stick_y * 1);

            rCS.setPosition((0.5 + (gamepad2.right_trigger / 2) - (gamepad2.left_trigger / 2)));
            lCS.setPosition(0.5 - (gamepad2.right_trigger / 2) + (gamepad2.left_trigger / 2));




            armPosition = armMotor.getCurrentPosition();
            telemetry.addLine("Arm Position: " + armMotor.getCurrentPosition());
            telemetry.addLine("lAS Position: " + lAS.getPosition());
            telemetry.addLine("rAS Position: " + rAS.getPosition());


            telemetry.update();

            sleep(25);

        }
    }
    public void evaluatePosition(int position, double power) throws InterruptedException{
        if (position == 1){
            moveArmTo(181, 0);
        }else if (position == 2){
            moveArmTo(181, 0);
        }else if (position == 3){
            moveArmTo(181, 0);
        }
    }
    public void moveArmTo(int position, double power) throws InterruptedException {
        armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        int leftPos = armMotor.getCurrentPosition();

        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
        while (armMotor.isBusy()){
            //auto code
        }
        armMotor.setPower(0);
    }
}