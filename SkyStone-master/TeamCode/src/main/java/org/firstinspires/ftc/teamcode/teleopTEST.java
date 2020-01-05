package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

// /n
//

@TeleOp(name = "Teleop-TEST",group = "Test")
public class teleopTEST extends LinearOpMode{
    private DcMotor[] arr = new DcMotor[4];

    //Drive Train
    private DcMotor RR; //Rear Right
    private DcMotor RL; //Rear Left
    private DcMotor FR; //Front Right
    private DcMotor FL; //Front Left
    //Arm Motors
    private DcMotor ySlide;
    private DcMotor xSlider;
    private DcMotor xSlidel;
    //Servo Nubs
    private Servo finger;




    @Override
    public void runOpMode() throws InterruptedException{

        RR = hardwareMap.dcMotor.get("rr");
        RL = hardwareMap.dcMotor.get("rl");
        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR = hardwareMap.dcMotor.get("fr");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL = hardwareMap.dcMotor.get("fl");


        arr[0] = FL;

        arr[1] = FR;
        arr[2] = RL;
        arr[3] = RR;



        ySlide = hardwareMap.dcMotor.get("y");

        xSlider = hardwareMap.dcMotor.get("xr");
        xSlidel = hardwareMap.dcMotor.get("xl");
        xSlidel.setDirection(DcMotorSimple.Direction.REVERSE);

        finger = hardwareMap.get(Servo.class, "finger");


                waitForStart();

        //below is what happens when you press start
        while (opModeIsActive()) {
            //Begin Gamepad "D"

            //strafing
            if(gamepad1.dpad_up){
                setPowers(new double[] {1, 1,
                                        1, 1});
            }
            if(gamepad1.dpad_down){
                setPowers(new double[]{-1, -1,
                                       -1, -1});
            }
            if(gamepad1.dpad_right){
                setPowers(new double[]{+1, -1,
                                       -1, +1});
            }
            if(gamepad1.dpad_left){
                setPowers(new double[]{-1, +1,
                                       +1, -1});
            }
            //triggers
            double rt = gamepad1.right_trigger/1.5;
            double lt = gamepad1.left_trigger/1.5;
            setPowers(new double[]{lt, rt,
                                   lt, rt});
            //sticks
            double rs = gamepad1.right_stick_y* -1;
            double ls = gamepad1.left_stick_y* -1;
            setPowers(new double[]{ls, rs,
                                   ls, rs});
            //End Gamepad "D"

            //Begin Gamepad "A"
            xSlider.setPower(gamepad2.right_stick_y/5);
            xSlidel.setPower(gamepad2.right_stick_y/5);

             ySlide.setPower(gamepad2.left_stick_y/1.15);
            //-0.2601
            if (dpadPressedOn(gamepad2)){
                ySlide.setPower(-0.2601);
            }

            if (gamepad2.a){
                finger.setPosition(0);
            }
            if (gamepad2.b){
                finger.setPosition(.33);
            }

            telemetry.addLine("Slide's Power: " + gamepad2.left_stick_y/1.15);
            telemetry.addLine("LeftfromBeginning: |" + FL.getCurrentPosition());
            telemetry.addLine("RightfromBeginning: |" + FR.getCurrentPosition());
            telemetry.update();

            //reset the motor's power
            setPowers(new double[]{0, 0,
                                   0, 0});
            sleep(3);

        }


    }
    public void setPowers(double[] args){
        for (int x = 0; x < args.length; x ++){
            arr[x].setPower(args[x]);
        }
    }
    public boolean dpadPressedOn(Gamepad gamepad){
        if (gamepad.dpad_down || gamepad.dpad_up || gamepad.dpad_left || gamepad.dpad_right){
            return true;
        } else {
            return false;
        }

    }




}