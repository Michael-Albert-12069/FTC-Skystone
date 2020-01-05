package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "R-SFP", group = "Autonomous")
public class Auto extends LinearOpMode {
    private DcMotor[] arr = new DcMotor[4];
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

    private Servo foundR;
    private Servo foundL;


    @Override
    public void runOpMode() throws InterruptedException{
        //initialize Drive-Train
        RR = hardwareMap.dcMotor.get("rr");
        RL = hardwareMap.dcMotor.get("rl");
        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR = hardwareMap.dcMotor.get("fr");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL = hardwareMap.dcMotor.get("fl");

        //assign the array's elements to motors
        arr[0] = FL;
        arr[1] = FR;
        arr[2] = RL;
        arr[3] = RR;

        //make all of the motors encoder-ized
        for (int x  = 0; x < arr.length; x ++){
            arr[x].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arr[x].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        finger = hardwareMap.get(Servo.class, "finger");
        foundL = hardwareMap.get(Servo.class, "foundL"); // up: 0.9 | 0.45
        foundR = hardwareMap.get(Servo.class, "foundR"); //     0.2 | 0.30




        //initialize the linear slide
        ySlide = hardwareMap.dcMotor.get("y");
        ySlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        xSlider = hardwareMap.dcMotor.get("xr");
        xSlidel = hardwareMap.dcMotor.get("xl");
        xSlidel.setDirection(DcMotorSimple.Direction.REVERSE);
        //    thumb = hardwareMap.servo.get("thumb");
        //    finger = hardwareMap.servo.get("finger");
//insert Ryer slam head on table
        releaseFoundation();
        finger.setPosition(0);


        waitForStart();
        driveForward((int) INtoCM(20));
        turnLeft((int)90/2.445);
        driveForward((int) INtoCM(62));
        turnRight((int) 90/2.445);
        driveForward((int) INtoCM(15));
        finger.setPosition(.33);
        sleep(100);
        finger.setPosition(.33);
        driveBackward(35);
        turnRight((int) 90/2.445);
        driveForward((int) INtoCM(67));
        liftSlide(10);
        holdSlide();
        turnLeft((int)90/2.445);
        finger.setPosition(0);
        releaseSlide();
        driveForward((int) INtoCM(15));
        grabFoundation();
        driveBackward((int)INtoCM(100));







        /*
        finger.setPosition(0);
        driveForward((int) INtoCM(15));
        turnLeft(90);
        driveForward((int) INtoCM(54));
        turnRight(90);
        driveForward((int) INtoCM(15));
        finger.setPosition(0);
        sleep(100);
        finger.setPosition(.33);
        sleep(100);
        */





    }
    public void driveForward(int cm) throws InterruptedException {
        for (int x  = 0; x < arr.length; x ++){
            arr[x].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }



        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        arr[1].setTargetPosition(cm * 13);
        arr[0].setTargetPosition(cm * 13);
        arr[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arr[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowers(new double[]{1, 1,
                               1, 1});
        while (arr[0].isBusy() && arr[1].isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void driveBackward(int cm) throws InterruptedException {
        for (int x  = 0; x < arr.length; x ++){
            arr[x].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }



        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        arr[1].setTargetPosition(cm * -13);
        arr[0].setTargetPosition(cm * -13);
        arr[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arr[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowers(new double[]{-1, -1,
                               -1, -1});
        while (arr[0].isBusy() && arr[1].isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void turnLeft(double degrees) throws InterruptedException {
        for (int x  = 0; x < arr.length; x ++){
            arr[x].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }



        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        setPositions(new int[]{(int)(degrees) * -13, (int)(degrees) * +13,
                               (int)(degrees) * -13, (int)(degrees) * +13});
        setPowers(new double[]{-1, 1,
                               -1, 1});
        while (arr[0].isBusy() && arr[1].isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void turnRight(double degrees) throws InterruptedException {
        for (int x  = 0; x < arr.length; x ++){
            arr[x].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }



        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        setPositions(new int[]{(int)(degrees) * +13, (int)(degrees) * -13,
                               (int)(degrees) * +13, (int)(degrees) * -13});
        setPowers(new double[]{1, -1,
                               1, -1});
        while (arr[0].isBusy() && arr[1].isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void stopDrive() throws InterruptedException {
        for (int x = 0; x < 4; x ++){
            arr[x].setPower(0);
        }
    }
    public static double INtoCM(double in){
        return 2.54 * in;
    }
    public void setPowers(double[] args){
        for (int x = 0; x < args.length; x ++){
            arr[x].setPower(args[x]);
        }
    }
    public void setPositions(int[] positions){
        for (int x = 0; x < 4; x ++){
            arr[x].setTargetPosition(positions[x]);
            arr[x].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void grabFoundation(){
        foundL.setPosition(0.45);
        foundR.setPosition(0.7);
    }
    public void releaseFoundation(){
        foundL.setPosition(0.9);
        foundR.setPosition(0.2);
    }
    public void liftSlide(int cm){
        ySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ySlide.setTargetPosition((int) Math.round(cm * 15.278));
        //6* Math.PI = 18 cm radius

        ySlide.setPower(-1);

    }
    public void holdSlide(){
        ySlide.setPower(-0.2601);

    }
    public void releaseSlide(){
        ySlide.setPower(0);

    }
}
