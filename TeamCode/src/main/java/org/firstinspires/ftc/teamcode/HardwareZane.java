package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Created by charliewu on 9/22/17.
 */

public class HardwareZane {
    /* Public OpMode members. */
    public DcMotor fl   = null;
    public DcMotor fr  = null;
    public DcMotor bl   = null;
    public DcMotor br  = null;
    public DcMotor lifter = null;
    public DcMotor extender = null;

    //public DcMotorController left = null;
    //public DcMotorController right = null;
    //public DcMotorController liftControl = null;

    //public ServoController servoController1 = null;

    public Servo dropperServo = null;
    public Servo flipperServo = null;
    public Servo leftHandTop = null;
    public Servo rightHandTop = null;
    public Servo leftHandBottom = null;
    public Servo rightHandBottom = null;
    public Servo relicGrabber = null;
    public Servo relicWrist = null;
    public Servo glyphFlipper = null;

    public ColorSensor jewelColor = null;

    public ColorSensor frColor = null;
    public ColorSensor blColor = null;

    public ModernRoboticsI2cGyro mainGyro = null;

    //public DeviceInterfaceModule dim = null;

    //public ModernRoboticsI2cCompassSensor compass = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareZane(){

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
        extender  = hwMap.dcMotor.get("extender");


        //left = hwMap.dcMotorController.get("leftControl");
        //right = hwMap.dcMotorController.get("rightControl");
        //liftControl = hwMap.dcMotorController.get("liftControl");

        //servoController1 = hwMap.servoController.get("servoControl1");

        dropperServo = hwMap.servo.get("dropperServo");
        flipperServo = hwMap.servo.get("flipperServo");
        leftHandTop = hwMap.servo.get("leftHandTop");
        rightHandTop = hwMap.servo.get("rightHandTop");
        leftHandBottom = hwMap.servo.get("leftHandBottom");
        rightHandBottom = hwMap.servo.get("rightHandBottom");
        relicGrabber = hwMap.servo.get("relicGrabber");
        relicWrist = hwMap.servo.get("relicWrist");
        glyphFlipper = hwMap.servo.get("glyphFlipper");

//        DcMotorEx a = (DcMotorEx)hwMap.get("temp");
//        a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        a.setMotorEnable();
//        a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        a.setTargetPositionTolerance(3);
//        a.setDirection(DcMotorSimple.Direction.FORWARD);
//        a.setTargetPosition(1400);
//        a.setVelocity(300, AngleUnit.DEGREES);


        jewelColor = hwMap.get(ModernRoboticsI2cColorSensor.class, "jewelColor");
        jewelColor.setI2cAddress(I2cAddr.create8bit(0x46));
//        frColor = hwMap.get(ModernRoboticsI2cColorSensor.class, "frColor");
//        frColor.setI2cAddress(I2cAddr.create8bit(0x44));
//        blColor = hwMap.get(ModernRoboticsI2cColorSensor.class, "blColor");
//        blColor.setI2cAddress(I2cAddr.create8bit(0x42));

        mainGyro = hwMap.get(ModernRoboticsI2cGyro.class, "mainGyro");

        //dim = hwMap.get(ModernRoboticsUsbDeviceInterfaceModule.class, "dim");

        //compass = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass");

        fl.setPower(sp);
        fr.setPower(sp);
        bl.setPower(sp);
        br.setPower(sp);
        lifter.setPower(sp);

        dropperServo.setPosition(0.5);
        flipperServo.setPosition(0.5);
        leftHandTop.setPosition(0.1);
        rightHandTop.setPosition(0.9);

        relicGrabber.setPosition(0.5);
        relicWrist.setPosition(0.5);

        glyphFlipper.setPosition(0);



        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public int getHeadingGyro(){
        return -mainGyro.getIntegratedZValue();
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



}
