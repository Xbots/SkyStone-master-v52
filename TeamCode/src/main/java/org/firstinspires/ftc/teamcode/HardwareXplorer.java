package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HardwareXplorer {
    /* Public OpMode members. */
    public DcMotor fl   = null;
    public DcMotor fr  = null;
    public DcMotor bl   = null;
    public DcMotor br  = null;
    public DcMotor lifter = null;
    public DcMotor collector = null;
    public Servo CollectorServo = null;
    public Servo TeamMarker = null;
    public Servo markerMover = null;
    public Servo intake = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo loader = null;
    public Servo intake1 = null;
    public Servo intake2 = null;

    public BNO055IMU revIMU = null;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareXplorer(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        double sp = 0;

        // Define and Initialize Motors
        fl   = hwMap.dcMotor.get("fl");
        fr  = hwMap.dcMotor.get("fr");
        bl   = hwMap.dcMotor.get("bl");
        br  = hwMap.dcMotor.get("br");
        lifter  = hwMap.dcMotor.get("lifter");
        markerMover = hwMap.servo.get("markerMover");
        leftClaw = hwMap.servo.get("leftClaw");
        rightClaw = hwMap.servo.get("rightClaw");
        loader = hwMap.servo.get("loader");
        collector = hwMap.dcMotor.get("collector");
        leftDistanceSensor =  hwMap.get(DistanceSensor.class, "leftSensor");
        rightDistanceSensor =  hwMap.get(DistanceSensor.class, "rightSensor");
        intake = hwMap.servo.get("intake");
        intake2 = hwMap.servo.get("intake2");
        //extender  = hwMap.dcMotor.get("extender");


        //CollectorServo = hwMap.servo.get("dropperServo");


//        DcMotorEx a = (DcMotorEx)hwMap.get("temp");
//        a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        a.setMotorEnable();
//        a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        a.setTargetPositionTolerance(3);
//        a.setDirection(DcMotorSimple.Direction.FORWARD);
//        a.setTargetPosition(1400);
//        a.setVelocity(300, AngleUnit.DEGREES);

        revIMU = hwMap.get(BNO055IMU.class, "gyro");
        ModernRoboticsI2cRangeSensor rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        //dim = hwMap.get(ModernRoboticsUsbDeviceInterfaceModule.class, "dim");

        //compass = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass");

        fl.setPower(sp);
        fr.setPower(sp);
        bl.setPower(sp);
        br.setPower(sp);
        lifter.setPower(sp);

        //where init used to be
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        revIMU.initialize(parameters);


        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoders()
    {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void encoderDrive(double flspeed, double frspeed, double blspeed, double brspeed, int fltarget, int frtarget, int bltarget, int brtarget) {
        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;

        // Determine new target position, and pass to motor controller
        newFLTarget = fl.getCurrentPosition()+fltarget;
        newFRTarget = fr.getCurrentPosition()+frtarget;
        newBLTarget = bl.getCurrentPosition()+bltarget;
        newBRTarget = br.getCurrentPosition()+brtarget;

        resetEncoders();

        fl.setPower(flspeed);
        fr.setPower(frspeed);
        bl.setPower(blspeed);
        br.setPower(brspeed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while(Math.abs(fl.getCurrentPosition())<Math.abs(fltarget) || Math.abs(fr.getCurrentPosition())<Math.abs(frtarget) || Math.abs(bl.getCurrentPosition())<Math.abs(bltarget) || Math.abs(br.getCurrentPosition())<Math.abs(brtarget))
        {
            if(!(Math.abs(fl.getCurrentPosition())<Math.abs(fltarget)))
            {
                fl.setPower(0);
            }
            if(!(Math.abs(fr.getCurrentPosition())<Math.abs(frtarget)))
            {
                fr.setPower(0);
            }
            if(!(Math.abs(bl.getCurrentPosition())<Math.abs(bltarget)))
            {
                bl.setPower(0);
            }
            if(!(Math.abs(br.getCurrentPosition())<Math.abs(brtarget)))
            {
                br.setPower(0);
            }
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void driveLimitless(double flspeed, double frspeed, double blspeed, double brspeed) {
        fl.setPower(flspeed);
        fr.setPower(frspeed);
        bl.setPower(blspeed);
        br.setPower(brspeed);
    }

    public void allStop()
    {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void moveArmLimitless(int direction){
        if(direction == 0){
            lifter.setPower(0);
        }
        if(direction == 1){
            lifter.setPower(0.3);
        }
        if(direction == 2){
            lifter.setPower(-0.3);
        }
    }

    public double getHeadingGyro(){
        return -revIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle;
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);
        // Reset the cycle clock for the next pass.
        period.reset();

    }
    public double getLeftDistance()
    {
        return leftDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double getRightDistance()
    {
        return rightDistanceSensor.getDistance(DistanceUnit.INCH);
    }


}
