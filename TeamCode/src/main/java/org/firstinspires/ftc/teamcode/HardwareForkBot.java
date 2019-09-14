package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by cwu on 10/8/2016
 */
public class HardwareForkBot {
    /* Public OpMode members. */
    public DcMotor fl   = null;
    public DcMotor fr  = null;
    public DcMotor bl   = null;
    public DcMotor br  = null;

    public DcMotor raiser = null;

    public Servo gripper = null;

    public ServoController sv1 = null;

    public DcMotorController arm = null;
    public DcMotorController front = null;
    public DcMotorController back = null;

    public DcMotor death = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareForkBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        fl   = hwMap.dcMotor.get("fl");
        fr  = hwMap.dcMotor.get("fr");
        bl   = hwMap.dcMotor.get("bl");
        br  = hwMap.dcMotor.get("br");

        raiser = hwMap.dcMotor.get("raiser");

        gripper = hwMap.servo.get("gripper");

        sv1 = hwMap.servoController.get("sv1");

        arm = hwMap.dcMotorController.get("arm");
        front = hwMap.dcMotorController.get("front");
        back = hwMap.dcMotorController.get("back");

        death = hwMap.dcMotor.get("death");

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        raiser.setPower(0);

        gripper.setPosition(MID_SERVO);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        raiser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void gripperPosition (double p_position)
    {
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION+0.1
                        , Servo.MAX_POSITION-0.1
                );
        gripper.setPosition(l_position);

    }
    public void resetEncoders()
    {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        //fl.setTargetPosition(newFLTarget);
        //fr.setTargetPosition(newFRTarget);
        //bl.setTargetPosition(newBLTarget);
        //br.setTargetPosition(newBRTarget);

        resetEncoders();

        // Turn On RUN_TO_POSITION
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setPower(flspeed);
        fr.setPower(frspeed);
        bl.setPower(blspeed);
        br.setPower(brspeed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while(Math.abs(fl.getCurrentPosition())<Math.abs(fltarget)
                || Math.abs(fr.getCurrentPosition())<Math.abs(frtarget)
                || Math.abs(bl.getCurrentPosition())<Math.abs(bltarget)
                || Math.abs(br.getCurrentPosition())<Math.abs(brtarget))
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
        //fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void driveLimitless(double flspeed, double frspeed, double blspeed, double brspeed) {

        //fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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