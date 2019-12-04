package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// \n


@TeleOp(name = "Teleop Comp 2",group = "TeleOp")
public class TeleOpComp2 extends LinearOpMode{
    DcMotor                 leftMotor, rightMotor;
    //TouchSensor             touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, rotation;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate, pidDrive;
    private DcMotor rLinMotor; //4
    private DcMotor lLinMotor; //1


    private Servo rAS;
    private Servo lAS;
    private Servo rCS;
    private Servo lCS;
    double lASclosePosition = 0.9;
    double lASopenPosition = 0.5;
    double rASclosePosition = 0.6;
    double rASopenPosition = 0.3;
    boolean armUp;



    @Override
    public void runOpMode() throws InterruptedException{
        rLinMotor = hardwareMap.get(DcMotor.class, "rArm");
        rLinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lLinMotor = hardwareMap.get(DcMotor.class, "lArm");
        lLinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lLinMotor.setDirection(DcMotor.Direction.REVERSE);

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


            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.right_stick_y);

            lLinMotor.setPower(gamepad2.right_stick_y/3);
            rLinMotor.setPower(gamepad2.right_stick_y/3);




            if (gamepad2.a) {//open
                lAS.setPosition(0.5);
                rAS.setPosition(0.3);
            }
            if (gamepad2.b) {//close
                lAS.setPosition(0.9+0.05);
                rAS.setPosition(0.6+0.05);
            }
            //sets the claw to open on A and close on B

            //   armMotor.setPower(gamepad2.right_stick_y * 1);

            rCS.setPosition((0.5 + (gamepad2.right_trigger / 2) - (gamepad2.left_trigger / 2)));
            lCS.setPosition(0.5 - (gamepad2.right_trigger / 2) + (gamepad2.left_trigger / 2));




            telemetry.addLine("lAS Position: " + lAS.getPosition());
            telemetry.addLine("rAS Position: " + rAS.getPosition());
            telemetry.addLine("Switch Location: " + position);
            telemetry.addLine("rCS Position: " + rCS.getPosition());
            telemetry.addLine("lCS Position: " + lCS.getPosition());

            telemetry.addLine("left slide Position: " + lLinMotor.getPowerFloat());
            telemetry.addLine("right slide Position: " + rLinMotor.getPowerFloat());






            telemetry.update();

            sleep(25);

        }
    }


}