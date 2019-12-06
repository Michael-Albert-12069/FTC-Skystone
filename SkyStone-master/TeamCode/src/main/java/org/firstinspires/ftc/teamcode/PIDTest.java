

// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="Drive Avoid Imu", group="Exercises")
//@Disabled
public class PIDTest extends LinearOpMode
{
    private Servo rAS;
    private Servo lAS;
    private Servo rCS;
    private Servo lCS;
    DcMotor                 leftMotor, rightMotor;
    //TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;
    private DcMotor rLinMotor; //4
    private DcMotor lLinMotor; //1

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        rLinMotor = hardwareMap.get(DcMotor.class, "rArm");
        rLinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lLinMotor = hardwareMap.get(DcMotor.class, "lArm");
        lLinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lLinMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        rAS = hardwareMap.servo.get("rightAS");
        rAS.setDirection(Servo.Direction.REVERSE);

        lAS = hardwareMap.servo.get("leftAS");
        rCS = hardwareMap.servo.get("rightCS");

        lCS = hardwareMap.servo.get("leftCS");
        lCS.setDirection(Servo.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to touch sensor.
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

        // drive until end of period.




        // Use gyro to drive in a straight line.
        correction = checkDirection();

        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();

        leftMotor.setPower(power - correction);
        rightMotor.setPower(power + correction);

        // We record the sensor values because we will test them in more than
        // one place with time passing between those places. See the lesson on
        // Timing Considerations to know why.




        // move to bridge.
        leftMotor.setPower(power*4);
        rightMotor.setPower(power*4);

        sleep(1750);

        // stop.
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // get on bridge
        rotate(45, power*2);





        //ram onto it
        leftMotor.setPower(power*5);
        rightMotor.setPower(power*5);

        sleep(975);

        // stop.
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        //straighten out
        rotate(45, power);

        //exit bridge
        leftMotor.setPower(power*4);
        rightMotor.setPower(power*4);

        sleep(600);

        // stop.
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        //open claw
        openServo();
        closeServo();
        openServo();

        //face bricks
        rotate(30 , power);

        //pull up to bricks
        leftMotor.setPower(power);
        rightMotor.setPower(power);

        sleep(1100);

        // stop.
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        closeServo();
        pull(1000);


        //lift arm
        lLinMotor.setPower(0.3);
        rLinMotor.setPower(0.3);

        sleep(500);

        lLinMotor.setPower(0);
        rLinMotor.setPower(0);






        //pull out of bricks
        leftMotor.setPower(-power*5);
        rightMotor.setPower(-power*5);

        sleep(1500);

        // stop.
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        //lift arm
        lLinMotor.setPower(0.3);
        rLinMotor.setPower(0.3);

        sleep(500);

        lLinMotor.setPower(0);
        rLinMotor.setPower(0);

        //turn to the foundation
        rotate(100, power * 1.5);

        //pull to reface foundation
        leftMotor.setPower(-power*5);
        rightMotor.setPower(-power*5);

        sleep(150);

        // stop.
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        rotate(25, power * 1.5);

        //pull into foundation
        leftMotor.setPower(power*5);
        rightMotor.setPower(power*5);

        sleep(250);

        // stop.
        leftMotor.setPower(0);
        rightMotor.setPower(0);


        push(1000);













        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void closeServo(){
        lAS.setPosition(0.9+0.05);
        rAS.setPosition(0.6+0.05);
    }
    public void openServo(){
        lAS.setPosition(0.5);
        rAS.setPosition(0.3);
    }
    public void pull(int millis){
        //+ is out
        //- is in
        rCS.setPosition(.3);
        lCS.setPosition(.6);
        sleep(millis);
        rCS.setPosition(.5);
        lCS.setPosition(.5);

    }
    public void push(int millis){
        rCS.setPosition(.6);
        lCS.setPosition(.3);
        sleep(millis);
        rCS.setPosition(.5);
        lCS.setPosition(.5);
    }

}

