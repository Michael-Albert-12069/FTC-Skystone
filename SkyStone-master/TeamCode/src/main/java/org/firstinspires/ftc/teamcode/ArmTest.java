package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Base64;

@TeleOp(name = "Arm Test",group = "TeleOp")
public class ArmTest extends LinearOpMode {
    private DcMotor armMotor;
    private Servo rAS;
    private Servo lAS;
    private Servo rCS;
    private Servo lCS;
    double thumbAngle = 0.0;

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


        waitForStart();



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


