package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Modified by isaiahturner on 9/15/18.
 */
/*4 motors for wheels
    1 pulley-use y to raise the lift b to lower the lift
    1 servo for claw- bumpers to open and close claw
    */
public class HardwareRover {
    /* Public OpMode members. */
    public DcMotor fl  = null;
    public DcMotor fr  = null;
    public DcMotor bl  = null;
    public DcMotor br  = null;
    public DcMotor lifter = null;//
    public DcMotor collector = null;
    public DcMotor extender = null;
    public DcMotor distributor = null;

    public BNO055IMU gyro = null;
    public Servo markerMover = null;
    public Servo intake = null;
    public Servo loader = null;
    public DistanceSensor leftDistanceSensor;
    //public DistanceSensor frontRightDistanceSensor;
    //public DistanceSensor backRightDistanceSensor;
    public ModernRoboticsI2cRangeSensor rangeSensor, otherRangeSensor;

    public DistanceSensor frontDistanceSensor;


    Orientation lastAngles = new Orientation();
    double globalAngle = 0;
    public static final double MID_SERVO       =  0.5 ;


    //public ServoController servoController1 = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRover(){

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
        collector = hwMap.dcMotor.get("collector");
        extender  = hwMap.dcMotor.get("extender");
        distributor  = hwMap.dcMotor.get("distributor");
        markerMover = hwMap.servo.get("markerMover");
        //leftDistanceSensor =  hwMap.get(DistanceSensor.class, "leftSensor");
        //frontRightDistanceSensor =  hwMap.get(DistanceSensor.class, "frontRightSensor");
        //backRightDistanceSensor =  hwMap.get(DistanceSensor.class, "rightSensor");
        leftDistanceSensor =  hwMap.get(DistanceSensor.class, "leftSensor");
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "MRrangeSensor");
        otherRangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "MRrangeOther");
        frontDistanceSensor = hwMap.get(DistanceSensor.class, "range");
        intake = hwMap.servo.get("intake");
        loader = hwMap.servo.get("loader");


        //claw = hwMap.servo.get("claw");
        BNO055IMU.Parameters parameters =  new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        gyro = hwMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        fl.setPower(sp);
        fr.setPower(sp);
        bl.setPower(sp);
        br.setPower(sp);
        lifter.setPower(sp);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    /**
     * Returns signed heading that can extend without limit (getIntegratedZValue on modern robotics
     * http://www.modernroboticsinc.com/Content/Images/uploaded/Sensors/Modern_Robotics_Gryo_Sensor-Steering_Tutorial.pdf)
     *
     * @return
     */
    public double getHeadingGyro(){
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
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
        //return 4;
    }
    public double getFrontRightDistance()
    {
        //return backRightDistanceSensor.getDistance(DistanceUnit.INCH);
        return otherRangeSensor.getDistance(DistanceUnit.INCH);

    }

    public double getBackRightDistance()
    {
        //return backRightDistanceSensor.getDistance(DistanceUnit.INCH);
        double x = rangeSensor.getDistance(DistanceUnit.INCH);
        if (x > 0)
            return x;
        else // if backSensor is not working, check the front sensor, as a backup so code will work
        {
                double y = otherRangeSensor.getDistance(DistanceUnit.INCH);
                if (y > 0)
                    return y;
                else
                    return 4; // does this help?
        }
    }

    public double getFrontDistance() { return frontDistanceSensor.getDistance(DistanceUnit.INCH);}

}
