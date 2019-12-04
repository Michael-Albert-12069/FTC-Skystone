package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "DriveForward", group = "Autonomous")
public class AutoStraight extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo thumb;
    private Servo finger;
    private int targetposition;
    //TouchSensor             touch;
    BNO055IMU               imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, rotation;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate, pidDrive;

    @Override
    public void runOpMode() throws InterruptedException{
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to REV Touch sensor.
        //touch = hardwareMap.touchSensor.get("touch_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 1);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        waitForStart();
        driveForward(40);







    }
    public void driveForward(int cm) throws InterruptedException {
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        int rightPos = rightMotor.getCurrentPosition();
        int leftPos = leftMotor.getCurrentPosition();

        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        rightMotor.setTargetPosition(rightPos + (10*cm));
        leftMotor.setTargetPosition(leftPos+ (10*cm));
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(1);
        leftMotor.setPower(1);
        while (leftMotor.isBusy() && rightMotor.isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void turnLeft(int degrees) throws InterruptedException {
        int cm =  degToCM(degrees);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        int rightPos = rightMotor.getCurrentPosition();
        int leftPos = leftMotor.getCurrentPosition();

        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        rightMotor.setTargetPosition(rightPos - (10*cm));
        leftMotor.setTargetPosition(leftPos+ (10*cm));
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(-1);
        leftMotor.setPower(1);
        while (leftMotor.isBusy() && rightMotor.isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void turnRight(int degrees) throws InterruptedException {
        int cm =  degToCM(degrees);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        int rightPos = rightMotor.getCurrentPosition();
        int leftPos = leftMotor.getCurrentPosition();
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //28.26 cm per revolution
        //288 ticks per revolution
        int conversionFactor = 10;
        rightMotor.setTargetPosition(rightPos + (10*cm));
        leftMotor.setTargetPosition(leftPos - (10*cm));
        rightMotor.setPower(1);
        leftMotor.setPower(-1);
        while (leftMotor.isBusy() && rightMotor.isBusy()){
            //nothing
        }
        stopDrive();
    }
    public void stopDrive() throws InterruptedException {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public static int degToCM(int deg){
        int cm = (int) Math.sqrt(120*deg);
        cm-=(0.2*deg);
        return cm;
    }
}
