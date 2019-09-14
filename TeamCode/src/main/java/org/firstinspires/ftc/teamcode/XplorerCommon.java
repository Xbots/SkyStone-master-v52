package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.HashMap;


public class XplorerCommon extends LinearOpMode{


    VuforiaLocalizer vuforia;

    HardwareRover robot = new HardwareRover();

    double offsetGyro = 0.0;
    double startHeading = 0.0;
    final double GYRO_THRESHOLD = 0.5;
    final double CORRECTION_MULTIPLIER = 0.02;
    final double ENC_PER_INCH = 44;
    double forwardSpeed = 0.7; //was 0.15
    double leftChange, rightChange;
    double changeNum;
    final double DS = 0.7;
    final double LIFT_ENCVAL = 21000;
    public Debugger comDbg = new Debugger("common debug");


    final double TURN_CORRECTION = 0.001;
    double turnSpeed = 0.7; //was 0.5
    double MIN_TURN_LIMIT = 0.1;
    double globalAngle, epochAngle, power = .30, correction;
    Orientation lastAngles = new Orientation();
    private static final String VUFORIA_KEY = "AYEB3rP/////AAABmVMCZTLJIE0TrkP2639XOkl5oPNywXyUOnq52N57nxQ2Q4KVO6xRk1CWWvTIbeZfVku0ISp4m3dPUpfORFAHlDqKOCdLUfRP78YbJexyDYJ+q2KQlap1/SH5sZi/llSpn5Y0b30k/VK/txgdo7TsyZBNZrcldc0KRiwo3NVeQwuVHjxFLJsU0P3MmwNDKZ5Fax3l1yglpGB5Ej2Vevu1gmVfGxxcnMaw0m89+olVLW/rKOB1mOjNC4nwOMsljk/5uY0OMBfkm14r6/HnvXk5bX/GLyBL2gPRdmIL5wtAn0JDjoa+RkhA930mTv0h4Mzh1gZ3PLwWf3Nnt3T5Qu7ffT/RX9zPJ7pKOlp9m4a0Hfs5";
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private final double ANGLE_THRESHOLD = 0.5;
    String pos;

    //Vuforia format
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    @Override
    public void runOpMode()
    {
        return;
    }
    public void initXplorer()
    {
        telemetry.addData("Init", "Waiting...");
        telemetry.update();
        robot.init(hardwareMap);

        DateFormat date = new SimpleDateFormat("MM-dd-yyyy HH:mm:ss");
        comDbg.openDebugFile("COMMON: " + date.format(Calendar.getInstance().getTime()));
        comDbg.setMsgPrefix("INFO");

        robot.markerMover.setPosition(0.57);
        robot.intake.setPosition(0.8);
        comDbg.debugMessage("L epochAngle at start: " +Double.toString(epochAngle%360));

        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        robot.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.fr.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bl.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.br.setDirection(DcMotorSimple.Direction.REVERSE);

        offsetGyro = robot.getHeadingGyro();
        startHeading = robot.getHeadingGyro()-offsetGyro;

        globalAngle = 0;
        lastAngles = new Orientation(); //reset both just in case

        robot.resetEncoders();

        telemetry.addData("Init", "done");
        telemetry.update();
    }

    public void landXplorer()
    {

        raiseLift();
        sleep(200);
        robot.resetEncoders();
        getAngle();
        comDbg.debugMessage("L epochAngle after landing: " +Double.toString(epochAngle%360));

    }

    public void strafeToGold(String pos)
    {
        drive( 0.7,robot.getHeadingGyro(), 7);
        robot.resetEncoders();

        if(pos.equals("left")) {
            while(opModeIsActive() && robot.fr.getCurrentPosition() < ENC_PER_INCH*15)
            {
                robot.driveLimitless(0.8, 0.8, -0.8, -0.8);
            }
            stopRobot();
            robot.resetEncoders();
        }

        if(pos.equals("center"))
        {
            while(opModeIsActive() && robot.br.getCurrentPosition() < ENC_PER_INCH*2)
            {
                robot.driveLimitless(-0.8, -0.8, 0.8, 0.8);
            }
            stopRobot();
            robot.resetEncoders();
        }

        if(pos.equals("right")) {
            while(opModeIsActive() && robot.br.getCurrentPosition() < ENC_PER_INCH*18)
            {
                robot.driveLimitless(-0.8, -0.8, 0.8, 0.8);
            }
            stopRobot();
            robot.resetEncoders();
        }
    }

    public void escapeLatch()
    {
        double a = getAngle();
        while(opModeIsActive() && robot.br.getCurrentPosition() < ENC_PER_INCH*5) //moving left
        {
            robot.driveLimitless(-0.8, -0.8, 0.8, 0.8);
        }
        robot.resetEncoders();
        stopRobot();
        while(opModeIsActive() && robot.br.getCurrentPosition() < ENC_PER_INCH*2)//moving up
        {
            robot.driveLimitless(-0.8, 0.8, -0.8, 0.8);
        }
        robot.resetEncoders();
        stopRobot();
        while(opModeIsActive() && robot.fr.getCurrentPosition() < ENC_PER_INCH*4)//moving right
        {
            robot.driveLimitless(0.8, 0.8, -0.8, -0.8);
        }
        stopRobot();
        robot.resetEncoders();
        comDbg.debugMessage("L epochAngle after escape: " +Double.toString(epochAngle%360));
        rotate( (int) epochAngle -(int)((((int)getAngle())%360)));
        comDbg.debugMessage("L epochAngle after escape correction: " +Double.toString(epochAngle%360));
    }

    public void lowerLift()
    {
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double timeStart = System.currentTimeMillis();
        robot.lifter.setPower(-1.0);
        while (opModeIsActive() && (robot.lifter.getCurrentPosition() > -LIFT_ENCVAL && (System.currentTimeMillis() - timeStart < 10000))) {
            //telemetry.addData("landing...", robot.lifter.getCurrentPosition());
            //telemetry.update();
        }
        robot.lifter.setPower(0);
        telemetry.addData("Xplorer Landed","");
        telemetry.update();
    }

    public void raiseLift()
    {
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double timeStart = System.currentTimeMillis();
        robot.lifter.setPower(1.0);
        while (opModeIsActive() && (robot.lifter.getCurrentPosition() < LIFT_ENCVAL && (System.currentTimeMillis() - timeStart < 10000))) {
            //telemetry.addData("landing...", robot.lifter.getCurrentPosition());
            //telemetry.update();
        }
        robot.lifter.setPower(0);
        telemetry.addData("Xplorer Landed","");
        telemetry.update();
    }

    public void raiseFull()
    {
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double timeStart = System.currentTimeMillis();
        robot.lifter.setPower(1.0);
        while (opModeIsActive() && (robot.lifter.getCurrentPosition() < (LIFT_ENCVAL+4000) && (System.currentTimeMillis() - timeStart < 10000))) {
            //telemetry.addData("landing...", robot.lifter.getCurrentPosition());
            //telemetry.update();
        }
        robot.lifter.setPower(0);
        telemetry.addData("Xplorer Landed","");
        telemetry.update();
    }

    public void lift(boolean extend, int encCount)
    {
        // if extend = true, lift extends
        // if extend = false, it contracts.
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (extend) {
            robot.lifter.setPower(-0.95);
            while (opModeIsActive() && robot.lifter.getCurrentPosition() > -encCount) {
            }
            ;
        }
        else {
            robot.lifter.setPower(0.95);
            while (opModeIsActive() && robot.lifter.getCurrentPosition() < encCount) {
            }
            ;
        }
        robot.lifter.setPower(0);
    }
    public void stopRobot()
    {
        robot.allStop();
        robot.resetEncoders();
        sleep(60);
    }
    public void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public String getGoldPosition(Debugger d) {
        String goldPosition = "";
        int imageMiddleCoordinate = -1;
        d.debugMessage("Op mode is: " + Boolean.toString(opModeIsActive()));
        int numtries = 0;
        boolean dummy = opModeIsActive();
        if (!dummy)
            dummy = true;
        if (dummy) {
            if (tfod != null) {
                tfod.activate();
            }
            while (dummy && numtries < 10) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {

                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    d.debugMessage("XXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n RUNNING MINERAL DETECTION");
                    d.debugMessage("# objects detected: " + updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0)
                    {
                        numtries += 1;
                    }
                    if (updatedRecognitions.size() >= 1) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        ArrayList<Recognition> goldMinerals = new ArrayList<>();
                        ArrayList<Recognition> silverMinerals = new ArrayList<>();
                        for (Recognition recognition : updatedRecognitions)
                        {
                            imageMiddleCoordinate = (int) recognition.getImageWidth() / 2;
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMinerals.add(recognition);
                            } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                silverMinerals.add(recognition);
                                silverMineral1X = (int) recognition.getLeft();
                            }
                        }
                        telemetry.addData("Number of gold minerals seen", goldMinerals.size());
                        d.debugMessage("Number of gold minerals seen: " + goldMinerals.size());
                        for(Recognition gold: goldMinerals){
                            telemetry.addData("Gold mineral bottom", gold.getBottom());
                            telemetry.addData("Gold mineral left", gold.getLeft());
                            telemetry.addData("Gold mineral confidence", gold.getConfidence());
                            telemetry.addData("---------", "");
                            d.debugMessage("Gold mineral bottom: "+gold.getBottom());
                            d.debugMessage("Gold mineral left: "+gold.getLeft());
                            d.debugMessage("Gold mineral confidence" + gold.getConfidence());
                            d.debugMessage("---------");
                        }
                        for(Recognition silver: silverMinerals){
                            telemetry.addData("Silver mineral bottom", silver.getBottom());
                            telemetry.addData("Silver mineral left", silver.getLeft());
                            telemetry.addData("Silver mineral confidence: ", silver.getConfidence());
                            telemetry.addData("---------", "");
                            d.debugMessage("Silver mineral bottom: "+silver.getBottom());
                            d.debugMessage("Silver mineral left: "+silver.getBottom());
                            d.debugMessage("Silver mineral confidence: " + silver.getConfidence());
                            d.debugMessage("---------");
                        }
                        float currentMax = -1;
                        for(Recognition gold: goldMinerals)
                        {
                            if(gold.getConfidence() < 0.54)
                            {
                                d.debugMessage("Likely false positive on gold... Ignoring");
                                continue;
                            }
                            if(gold.getBottom() < 80)
                            {
                                d.debugMessage("Gold mineral likely in crater... Ignoring");
                                continue;
                            }
                            if(gold.getBottom() > currentMax ) {
                                goldMineralX = (int) gold.getLeft();
                                telemetry.addData("Chosen gold mineral bottom", gold.getBottom());
                                telemetry.addData("Chosen gold mineral left", goldMineralX);
                                telemetry.addData("Chosen gold mineral confidence: ", gold.getConfidence());
                                telemetry.addData("---------", "");
                                d.debugMessage("Chosen gold mineral bottom: "+gold.getBottom());
                                d.debugMessage("Chosen gold mineral left: "+goldMineralX);
                                d.debugMessage("Chosen gold mineral confidence: " + gold.getConfidence());
                                d.debugMessage("---------");
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1) {//if both are seen
                            d.debugMessage("Both mineral types have been seen.");
                            d.debugMessage("Gold mineral X: " + goldMineralX);
                            d.debugMessage("Silver mineral X: " + silverMineral1X);
                            if (goldMineralX < silverMineral1X) {
                                goldPosition = "center";
                                telemetry.addData("Gold Mineral Position", goldPosition);
                            } else if (goldMineralX > silverMineral1X) {
                                goldPosition = "right";
                                telemetry.addData("Gold Mineral Position", goldPosition);
                            }
                        }
                        else if (silverMineral1X != -1) { //only silvers are seen
                            d.debugMessage("Only silver minerals seen.");
                            goldPosition = "left";
                            telemetry.addData("Gold Mineral Position", goldPosition);
                        }
                        else if (goldMineralX != -1 && imageMiddleCoordinate != -1) //gold is seen; no silver -- backup
                        {
                            d.debugMessage("Only gold has been seen --- using backup.");
                            if(goldMineralX > imageMiddleCoordinate)
                                goldPosition = "right";
                            else
                                goldPosition = "center";
                        }

                    }
                    //telemetry.update();
                }
                if(!goldPosition.equals("") || numtries >= 10)
                    break;
            }
        }
        if (numtries >= 10)
        {
            goldPosition = "left";
        }
        d.debugMessage("FINAL GOLD POSITION: " + goldPosition);
        d.debugMessage("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");

        return goldPosition;
    }

    public void dropMarker()
    {
        robot.markerMover.setPosition(0.01);

    }

    public void drive(double forwardSpeed, double startHeading, double dist){
        robot.resetEncoders();
        //telemetry.addData("moving forward...", dist);
        //telemetry.update();
        dist = dist * ENC_PER_INCH;

        while(opModeIsActive()  && robot.fr.getCurrentPosition() < dist) {
            moveForward (forwardSpeed, startHeading);
        }
        stopRobot();
    }
    public void driveNoStop(double forwardSpeed, double startHeading, double dist){
        robot.resetEncoders();
        //telemetry.addData("moving forward...", dist);
        //telemetry.update();
        dist = dist * ENC_PER_INCH;

        while(opModeIsActive()  && robot.fr.getCurrentPosition() < dist) {
            moveForward (forwardSpeed, startHeading);
        }
    }
    public void driveSensor(double forwardSpeed, double startHeading){
        robot.resetEncoders();
        //telemetry.addData("moving forward limitlessly...", "");
        //telemetry.update();
        /*while(opModeIsActive()  && robot.rangeSensor.getDistance(DistanceUnit.CM) > 2) {
            moveForward (forwardSpeed, startHeading);
        }*/
        stopRobot();
    }
    public void driveback(double backwardSpeed, double startHeading, double dist){
        backwardSpeed = Math.abs(backwardSpeed);
        robot.resetEncoders();
        //telemetry.addData("moving backward...", dist);
        //telemetry.update();
        dist = dist * ENC_PER_INCH;

        while(opModeIsActive()  && robot.fr.getCurrentPosition() > -dist) {
            moveBackward (backwardSpeed, startHeading);
        }
        stopRobot();
    }

    public void moveForward(double forwardSpeed, double startHeading){
        leftChange = forwardSpeed;
        rightChange = forwardSpeed;
        if(robot.getHeadingGyro() - offsetGyro - startHeading > GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getHeadingGyro() - offsetGyro - startHeading);
            rightChange += CORRECTION_MULTIPLIER * changeNum;
        }else if(robot.getHeadingGyro() - offsetGyro - startHeading < -GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getHeadingGyro() - offsetGyro - startHeading);
            leftChange += CORRECTION_MULTIPLIER * changeNum;
        }else{
            //robot is driving within acceptable range
        }
        robot.driveLimitless(-leftChange, rightChange, -leftChange, rightChange);
        //telemetry.addData("heading: ", robot.getHeadingGyro());
        //telemetry.addData("left adj: ", leftChange);
        //telemetry.addData("right adj: ", rightChange);
        //telemetry.update();
        //comDbg.debugMessage("MvFwd: left Change, right Change: " + Double.toString(leftChange) +", " +Double.toString(rightChange));

    }

    public void moveBackward(double backwardSpeed, double startHeading){
        telemetry.addData("movingBack: ", startHeading);
        telemetry.update();
        leftChange = backwardSpeed;
        rightChange = backwardSpeed;
        if(robot.getHeadingGyro() -offsetGyro - startHeading > GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getHeadingGyro() -offsetGyro - startHeading);
            leftChange += CORRECTION_MULTIPLIER * changeNum;
        }else if(robot.getHeadingGyro() -offsetGyro - startHeading < -GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getHeadingGyro() -offsetGyro - startHeading);
            rightChange += CORRECTION_MULTIPLIER * changeNum;
        }else{
            //robot is driving within acceptable range
        }
        robot.driveLimitless(leftChange, -rightChange, leftChange, -rightChange);
        //telemetry.addData("heading: ", robot.getHeadingGyro());
        //telemetry.addData("left adj: ", leftChange);
        //telemetry.addData("right adj: ", rightChange);
        //telemetry.update();
    }

    public void turnPoint(double turnTarget, double offsetGyro, boolean right){
        // if right <turnTarget else > turnTarget ( turnTarget is -ve
        //bool
        if (right) {
            while(opModeIsActive() && (robot.getHeadingGyro()-offsetGyro)%360<turnTarget) {
                turnSpeed = (turnTarget - (robot.getHeadingGyro() - offsetGyro) % 360) * TURN_CORRECTION + MIN_TURN_LIMIT;
                if (turnSpeed < 0.2 ) turnSpeed = 0.2;
                robot.driveLimitless(-turnSpeed, -turnSpeed, -turnSpeed, -turnSpeed);
                telemetry.addData("gyro heading: ", robot.getHeadingGyro());
                telemetry.update();
            }
        }

        else
        {
            while(opModeIsActive() && (robot.getHeadingGyro()-offsetGyro)%360>turnTarget) {
                turnSpeed = (turnTarget - (robot.getHeadingGyro() - offsetGyro) % 360) * TURN_CORRECTION + MIN_TURN_LIMIT;
                if (turnSpeed < 0.2 ) turnSpeed = 0.2;
                robot.driveLimitless(turnSpeed, turnSpeed, turnSpeed, turnSpeed);
                telemetry.addData("gyro heading: ", robot.getHeadingGyro());
                telemetry.update();
            }
        }

        robot.driveLimitless(0, 0, 0, 0);
        stopRobot();
    }
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        epochAngle += deltaAngle;


        lastAngles = angles;

        return globalAngle;
    }


    public double checkDirection()
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

    public String positionEstimate(Debugger d){
        String pos1 = "", pos2 = "", pos3 = "", finalPos = "";
        pos1 = getGoldPosition(d);
        d.debugMessage("Gold position without flash is: " + pos1);
        CameraDevice.getInstance().setFlashTorchMode(true);
        pos2 = getGoldPosition(d);
        d.debugMessage("Gold position with flash is: " + pos2);
        telemetry.addData("Poses:",pos1+" "+pos2);
        telemetry.addData("Getting gold position is done", "press start");
        telemetry.update();

        waitForStart();
        landXplorer();
        if(pos1.equals(pos2)) //if first two positions agree, use them as final
        {
            finalPos = pos1;
            d.debugMessage("Pos 1 and 2 are equal; setting final pos to: " + pos1);
        }
        else {
            pos3 = getGoldPosition(d);
            d.debugMessage("Pos 1 and 2 are different; gold position from ground is: " + pos3);
            if(pos3.equals(pos1)) // if these two agree
            {
                finalPos = pos3;
                d.debugMessage("Gold position from ground and without flash agree; setting final pos to: " + pos3);
            }
            else if (pos3.equals(pos2)) //if these two agree
            {
                finalPos = pos2;
                d.debugMessage("Gold position from ground and with flash agree; setting final pos to: " + pos2);
            }
            else //nothing agrees
            {
                finalPos = pos3;
                d.debugMessage("Nothing agrees; setting final pos to: " + pos3);
            }
        }
        return finalPos;
    }
    public void rotate(int degrees)
    {
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        // rotate until turn is completed.
        //power of 0.1 is good for most turns; leaves us with 5-6 seconds on left, 8-9 on others
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            //while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {
                //telemetry.addData("Current heading:", getAngle());
                //telemetry.addData("Degree", degrees);
                power = (degrees - (robot.getHeadingGyro() - offsetGyro) % 360) * TURN_CORRECTION + MIN_TURN_LIMIT;
                if (power < 0.2 ) power = 0.2;
                //telemetry.addData("Power", power);
                //telemetry.update();
                //power = 0.1;
                robot.driveLimitless(-power, -power, -power, -power);
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                //telemetry.addData("Current heading:", getAngle());
                //telemetry.addData("Degree", degrees);
                //telemetry.update();
                power = (degrees - (getAngle() - offsetGyro) % 360) * TURN_CORRECTION + MIN_TURN_LIMIT;
                if (power < 0.2 ) power = 0.2;
                //telemetry.addData("Power", power);
                //telemetry.update();
                //power = 0.1;
                robot.driveLimitless(power, power, power, power);
            }

        // turn the motors off.
        robot.driveLimitless(0, 0, 0, 0);
        stopRobot();
        // wait for rotation to stop
        // reset angle tracking on new heading.
        resetAngle();
        //sleep(200);// this can be removed. StopRobot has a 100ms sleep anyways.
    }
    public void resetAngle()
    {
        lastAngles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public void travelAlongWall(double speed, double distToTravel, double distFromWall, String wallSide, String direction, Debugger d)
    {
        double WALL_CORRECTION = 0.01;
        distToTravel = distToTravel * ENC_PER_INCH;
        double currentDist;
        double startHeading = robot.getHeadingGyro();

        while(opModeIsActive() && ( ( robot.fl.getCurrentPosition() < distToTravel && direction.equals("forward") )
                || ( ( robot.fr.getCurrentPosition() > -distToTravel && direction.equals("back") ) ))) {
            leftChange = speed;
            rightChange = speed;
            if(wallSide.equals("left")) {
                currentDist = robot.getLeftDistance();
            }
            else {
                currentDist = robot.getBackRightDistance();
            }
            double error = currentDist - distFromWall;
            d.debugMessage("Error value: "+error);

            if (error > 0) { //too far from the wall
                telemetry.addData("Too far from wall; error:", error);
                if (wallSide.equals("right"))//turn right
                {
                    if(direction.equals("forward"))
                        leftChange += WALL_CORRECTION * Math.abs(error);
                    else
                        rightChange += WALL_CORRECTION * Math.abs(error);
                } else if (wallSide.equals("left"))//turn left
                {
                    if(direction.equals("forward"))
                        rightChange += WALL_CORRECTION * Math.abs(error);
                    else
                        leftChange += WALL_CORRECTION * Math.abs(error);
                }
            }
            else if (error < 0) { //too close to the wall
                telemetry.addData("Too close to wall; error:", error);
                if (wallSide.equals("right"))//turn left
                {
                    if(direction.equals("forward"))
                        rightChange += WALL_CORRECTION * Math.abs(error);
                    else
                        leftChange += WALL_CORRECTION * Math.abs(error);
                } else if (wallSide.equals("left"))//turn right
                {
                    if(direction.equals("forward"))
                        leftChange += WALL_CORRECTION * Math.abs(error);
                    else
                        rightChange += WALL_CORRECTION * Math.abs(error);
                }
            } else {
                //robot is driving within acceptable range
            }
            if(direction.equals("forward")) {
                robot.driveLimitless(-leftChange, rightChange, -leftChange, rightChange);
                //moveForward(0.7, startHeading);
            }
            else{
                robot.driveLimitless(leftChange, -rightChange, leftChange, -rightChange);
                //moveBackward(0.7, startHeading);

            }
            telemetry.addData("heading: ", robot.getHeadingGyro());
            telemetry.addData("left adj: ", leftChange);
            telemetry.addData("right adj: ", rightChange);
            telemetry.update();
            d.debugMessage("Heading: " + robot.getHeadingGyro());
            d.debugMessage("Left change: " + leftChange);
            d.debugMessage("Right change: " + rightChange);

        }
    }

    public void alignToWall(String wallSide, Debugger d)
    {
        if (wallSide == "left")
            return;
        d.debugMessage("alignToWall");

        while (true) {
            double x = robot.getFrontRightDistance();
            double y = robot.getBackRightDistance();
            d.debugMessage("Front " + x + " Back: " + y);

            if ((x > 30) ||  (y > 30)) {
                d.debugMessage("  >30inches to wall  ");
                break;
            }

            if (((x-y) > 1) || ( (y-x) > 1)) {
                d.debugMessage("  >rotate  " + (int)(y-x));
                rotate( 2*((int)(y-x)));
            }
            else break;
        };
    }


    public void straightenToWall(String wallSide, Debugger d)
    {
        double frontSum = 0, backSum = 0;


        if (wallSide == "left")
            return;

        double x = robot.getFrontRightDistance();
        double y = robot.getBackRightDistance();
        d.debugMessage("Front right distance: " + x);
        d.debugMessage("Back right distance: " + y);
        if (x >0 && y > 0 && x < 16 && y < 16) {
            for (int i = 0; i < 5; i++) {
                frontSum += robot.getFrontRightDistance();
                backSum += robot.getBackRightDistance();
            }
            if (frontSum / 5 > 100 || backSum / 5 > 100) {
                d.debugMessage("Front right distance: " + robot.getFrontRightDistance());
                d.debugMessage("Back right distance: " + robot.getBackRightDistance());
                d.debugMessage("One of the distance sensors is not working. Unable to straighten robot.");
            } else {
                if (frontSum > backSum) // front of robot is further away; need to turn right
                    // using diff powers to right and left wheels rather than rotate
                    while ((robot.getFrontRightDistance() - robot.getBackRightDistance()) > 3) { //was 4
                        robot.driveLimitless(-0.7, -0.5, -0.7, -0.5);  // left 0.7 right 0.5
                        //rotate(-6);
                        d.debugMessage("Front right distance: " + robot.getFrontRightDistance());
                        d.debugMessage("Back right distance: " + robot.getBackRightDistance());
                    }

                else if (frontSum < backSum) // back of robot is further away; need to turn left
                    while (-1.0*(robot.getFrontRightDistance() - robot.getBackRightDistance()) > 3) { // was 4
                        // using diff powers to right and left wheels rather than rotate
                        robot.driveLimitless(0.5, 0.7, 0.5, 0.7);  // left 0.5 right 0.7
                        //rotate(6);
                        d.debugMessage("Front right distance: " + robot.getFrontRightDistance());
                        d.debugMessage("Back right distance: " + robot.getBackRightDistance());
                    }
            }
        }
    }

    public void strafeRight(double speed, double distFromWall, Debugger d)
    {
        while (robot.getBackRightDistance() > (distFromWall +2))
        {
            robot.driveLimitless(-speed, -speed, speed, speed);
        }
    }

    public void strafeLeft(double speed, double distFromWall, Debugger d)
    {
        while (robot.getLeftDistance() > (distFromWall +2))
        {
            robot.driveLimitless(speed, speed, -speed, -speed);
        }
    }

    public void wallStrafe(double speed, double distFromWall, String wallSide, String direction, Debugger d)
    {
        double currentDist;
        telemetry.addData("distance:", robot.getFrontDistance());
        telemetry.update();
        if (wallSide.equals("left")) {
            currentDist = robot.getLeftDistance();
        } else {
            currentDist = robot.getBackRightDistance();
        }
        double error = currentDist - distFromWall;
        d.debugMessage("Error value: " + error);
        d.debugMessage("Front left current pos: " + robot.fl.getCurrentPosition());
        d.debugMessage("Front right current pos: " + robot.fr.getCurrentPosition());
        d.debugMessage("Distance from front: " + robot.getFrontDistance());
        //d.debugMessage("Distance from front: " + robot.rangeSensor.getDistance(DistanceUnit.CM));
        if(Math.abs(error) > 3) {
            if (error > 0) { //too far from the wall
                telemetry.addData("Too far from wall; error:", error);
                d.debugMessage("Too far from wall; error:" + Double.toString(error));

                if (wallSide.equals("right"))//strafe right
                {
                    while (robot.getBackRightDistance() > (distFromWall +2))
                    {
                        d.debugMessage("Too far. Strafing right(IN): " +Double.toString(robot.getBackRightDistance()));
                        robot.driveLimitless(-speed, -speed, speed, speed);
                    }
                } else if (wallSide.equals("left"))//turn left
                {
                    while (robot.getLeftDistance() < (distFromWall-2) )
                    {
                        d.debugMessage("Too far. Strafing left(IN): " +Double.toString(robot.getBackRightDistance()));
                        robot.driveLimitless(speed, speed, -speed, -speed);
                    }
                }
            } else if (error < 0) { //too close to the wall
                telemetry.addData("Too close to wall; error:", error);
                d.debugMessage("Too close to wall; error:" + Double.toString(error));
                if (wallSide.equals("right"))//turn left
                {
                    // limit this to 4" since it is on the away side, feedback loop is bad
                    while (robot.getBackRightDistance() < (distFromWall - 2)) {
                        robot.driveLimitless(speed, speed, -speed, -speed);
                    }
                    d.debugMessage("Too Close. Strafing left.");
                } else if (wallSide.equals("left"))//turn right
                {
                    while (robot.getLeftDistance() < (distFromWall + 2)) {
                        robot.driveLimitless(-speed, -speed, speed, speed);
                    }
                    d.debugMessage("Too Close. Strafing right.");
                }
            }
        }
        else {
            if (direction.equals("forward")) {
                robot.driveLimitless(-speed, speed, -speed, speed);
                d.debugMessage("Moving forward.");
            }
            if (direction.equals("back")) {
                robot.driveLimitless(speed, -speed, speed, -speed);
                d.debugMessage("Moving back.");
            }
        }
    }


    public void FollowAlongWall(double speed, double distToTravel, double distFromWall, String wallSide, String direction, Debugger d)
    {
        distToTravel = distToTravel * ENC_PER_INCH;

        robot.resetEncoders();
        telemetry.addData("moving forward limitlessly...", "");
        telemetry.update();
        //straightenToWall(wallSide, d);
        alignToWall(wallSide, d);
        while(opModeIsActive()) {
            if (direction.equals("forward"))
                while (robot.fr.getCurrentPosition() < distToTravel ) //13 gets caught sometimes
                    robot.driveLimitless(-speed, speed, -speed, speed);

            if (direction.equals("back"))
                while ( robot.fl.getCurrentPosition() < distToTravel ) //13 gets caught sometimes
                    robot.driveLimitless(speed, -speed, speed, -speed);

            if (direction.equals("forward") && robot.fr.getCurrentPosition() >= distToTravel) {
                telemetry.addData("Follow along wall", "stopping forward movement");
                break;
            }
            if (direction.equals("back") && robot.fl.getCurrentPosition() >= distToTravel) {
                telemetry.addData("Follow along wall", "stopping backward movement");
                break;
            }
        }
        telemetry.addData("stopped", "");
        telemetry.addData("Front distance: ", robot.getFrontDistance());
        telemetry.update();
        stopRobot();
    }

    public void strafeAlongWall(double speed, double distToTravel, double distFromWall, String wallSide, String direction, Debugger d) {
        distToTravel = distToTravel * ENC_PER_INCH;

        robot.resetEncoders();
        telemetry.addData("moving forward limitlessly...", "");
        telemetry.update();
        //straightenToWall(wallSide, d);
        alignToWall(wallSide, d);
        while(opModeIsActive()) {
            while ( ((robot.fr.getCurrentPosition() < distToTravel && direction.equals("forward")) //13 gets caught sometimes
                    || (robot.fl.getCurrentPosition() < distToTravel && direction.equals("back")))) {
                wallStrafe(speed, distFromWall, wallSide, direction, d);
            }
            if (direction.equals("forward") && robot.fr.getCurrentPosition() >= distToTravel) {
                telemetry.addData("Strafe along wall", "stopping forward movement");
                break;
            }
            if (direction.equals("back") && robot.fl.getCurrentPosition() >= distToTravel) {
                telemetry.addData("Strafe along wall", "stopping backward movement");
                break;
            }
        }
        telemetry.addData("stopped", "");
        telemetry.addData("Front distance: ", robot.getFrontDistance());
        telemetry.update();
        stopRobot();
    }
    public void strafeAlongWallNoForwardSensor(double speed, double distToTravel, double distFromWall, String wallSide, String direction, Debugger d)
    {
        distToTravel = distToTravel * ENC_PER_INCH;

        robot.resetEncoders();
        telemetry.addData("moving forward limitlessly...", "");
        telemetry.update();
        straightenToWall(wallSide, d);
        while (opModeIsActive() && ((robot.fl.getCurrentPosition() < distToTravel && direction.equals("back"))
                || ((robot.fl.getCurrentPosition() > -distToTravel && direction.equals("forward"))))) {
            wallStrafe(speed, distFromWall, wallSide, direction, d);
        }
        telemetry.addData("stopped", "stopped");
        telemetry.update();
        //stopRobot();
    }
    public void sampleGoldDepot(String pos, Debugger d)
    {
        robot.resetEncoders();
        if(pos.equals("left")) {
            d.debugMessage("Gold is on the left.");
            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*3)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Drove forward 4 inches.");
            stopRobot();
            rotate(38);
            d.debugMessage("Rotated 38 deg left.");
            //stopRobot();

            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*28)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Drove forward 28 inches.");
            stopRobot();
            rotate(-90);
            d.debugMessage("Rotated 90 deg right.");
            //stopRobot();

            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*36))) //was 35
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Moved forward 36 inches.");
            stopRobot();
            rotate(87); //was 85
            d.debugMessage("Rotated 87 deg left.");
            //stopRobot();
        }
        if(pos.equals("center"))
        {
            d.debugMessage("Gold is in the center.");
            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*46)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Moved forward 46 inches.");
            stopRobot();
            rotate(37 );
            d.debugMessage("Rotated 37 deg left.");
            while(opModeIsActive() && robot.br.getCurrentPosition() < ENC_PER_INCH*5)
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Moved forward 8 inches.");
            stopRobot();
            while(opModeIsActive() && (robot.bl.getCurrentPosition() < (ENC_PER_INCH*4)))
            {
                robot.driveLimitless(DS, -DS, DS, -DS);
            }
            d.debugMessage("Moved backward 4 inches.");
            //sleep(200);  // do we need StopRobot() or sleep here? Motors are not reset.
        }
        if(pos.equals("right")) {
            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*3)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Moved forward 3 inches.");
            stopRobot();
            rotate(-41);
            d.debugMessage("Rotated 41 deg right.");
            //stopRobot();

            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*30)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Moved forward 30 inches.");
            stopRobot();
            while(opModeIsActive() && (robot.br.getCurrentPosition() > (ENC_PER_INCH*4)))
            {
                robot.driveLimitless(DS,-DS,DS,-DS);
            }
            d.debugMessage("Moved backward 4 inches.");
            stopRobot();
            rotate(88);
            d.debugMessage("Rotated 88 deg left.");
            //stopRobot();

            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*33)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Moved forward 33 inches.");

            stopRobot();
            while(opModeIsActive() && (robot.bl.getCurrentPosition() < (ENC_PER_INCH*4)))
            {
                robot.driveLimitless(DS, -DS, DS, -DS);
            }
            d.debugMessage("Moved forward 4 inches.");
            // need StopRobot() before sleep below?
            //sleep(200);
        }
        stopRobot();
    }
    public void sampleGoldDepotAndGoBack(String pos, Debugger d)
    {
        robot.resetEncoders();
        if(pos.equals("left")) {
            d.debugMessage("Gold is on the left.");
            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*6)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Drove forward 6 inches.");
            stopRobot();
            rotate(38);
            d.debugMessage("Rotated 38 deg left.");
            stopRobot();

            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*28)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Drove forward 28 inches.");
            stopRobot();
            while(opModeIsActive() && (robot.br.getCurrentPosition() > -(ENC_PER_INCH*28)))
            {
                robot.driveLimitless(DS, -DS, DS, -DS);
            }
            d.debugMessage("Drove backward 28 inches.");
            stopRobot();
        }
        if(pos.equals("center"))
        {
            d.debugMessage("Gold is in the center.");
            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*46)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Moved forward 46 inches.");
            stopRobot();
            while(opModeIsActive() && (robot.br.getCurrentPosition() > -(ENC_PER_INCH*46)))
            {
                robot.driveLimitless(DS, -DS, DS, -DS);
            }
            d.debugMessage("Moved backward 46 inches.");
            stopRobot();
            sleep(200);
        }
        if(pos.equals("right")) {
            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*6)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Moved forward 6 inches.");
            stopRobot();
            rotate(-41);
            d.debugMessage("Rotated 41 deg right.");
            stopRobot();

            while(opModeIsActive() && (robot.br.getCurrentPosition() < (ENC_PER_INCH*26)))
            {
                robot.driveLimitless(-DS, DS, -DS, DS);
            }
            d.debugMessage("Moved forward 26 inches.");
            stopRobot();
            while(opModeIsActive() && (robot.br.getCurrentPosition() > -(ENC_PER_INCH*26)))
            {
                robot.driveLimitless(DS, -DS, DS, -DS);
            }
            d.debugMessage("Moved forward 26 inches.");
            stopRobot();
            sleep(200);

        }
    }
    public void sampleGoldGyroDepot(String pos, Debugger d)
    {
        globalAngle = 0;
        lastAngles = new Orientation(); //reset both just in case
        //pos = getGoldPosition();
        telemetry.addData("Gold position", pos);
        telemetry.update();

        robot.resetEncoders();

        if (pos.equals("")) {
            pos = "center";
            d.debugMessage("DETECTION FAILED: FAKING GOLD POS AS CENTER.");
            telemetry.addData("FAILED DETECTION", pos);
            telemetry.update();
        }


        if(pos.equals("left")) {

            drive(DS, robot.getHeadingGyro(), 6);
            stopRobot();
            rotate(35);
            stopRobot();
            drive(DS, robot.getHeadingGyro(), 31);
            stopRobot();
            rotate(-90);
            stopRobot();
            drive(DS, robot.getHeadingGyro(), 38);
            stopRobot();
        }
        if(pos.equals("center"))
        {
            drive(DS, robot.getHeadingGyro(), 46);
            stopRobot();
            rotate(32);
            while(opModeIsActive() && robot.br.getCurrentPosition() < ENC_PER_INCH*8)
            {
                robot.driveLimitless(-DS, -DS, DS, DS);
            }
            stopRobot();
            driveback(DS, robot.getHeadingGyro(), 8);
        }
        if(pos.equals("right")) {
            drive(DS, robot.getHeadingGyro(), 6);
            rotate(-41);
            stopRobot();
            drive(DS, robot.getHeadingGyro(), 31);
            rotate(88);
            stopRobot();
            drive(DS, robot.getHeadingGyro(), 33);
            stopRobot();
        }
    }

    public void justSample(String pos, Debugger d)
    {
        globalAngle = 0;
        lastAngles = new Orientation(); //reset both just in case
        if (pos.equals(""))
        {
            pos = "center";
            d.debugMessage("DETECTTION FAILED: FAKING GOLD POS AS CENTER.");
            telemetry.addData("FAILED DETECTION", pos);
            telemetry.update();
        }

        telemetry.addData("Gold position", pos);
        telemetry.update();

        if(pos.equals("left")) {
            rotate(27); //was 30 before
            d.debugMessage("Gold is on the left; rotating 27 deg left");
        }
        if(pos.equals("center"))
        {
            d.debugMessage("Gold is in the center; not rotating");
        }
        if(pos.equals("right")) {
            rotate(-27);
            d.debugMessage("Gold is on the right; rotating 35 deg right");
        }

        drive(0.4, robot.getHeadingGyro(), 22);// push the mineral
        d.debugMessage("Moving forward 16 inches to hit mineral.");
        sleep(200);

    }

    public void returnToBase(String pos)
    {
        // position the robot to perpendicular to lander, face the wall at 45
        if(pos.equals("left")) {
            //driveback(DS, robot.getHeadingGyro(), 4); // get the block out of the way
            rotate(28);
            alignToWall("right", comDbg);
            strafeRight(0.3,5, comDbg);
            FollowAlongWall(.7, 40, 4,"right", "back", comDbg);
            //drive(DS,robot.getHeadingGyro(),40);
            comDbg.debugMessage("L epochAngle after knocking & turning: " +Double.toString(epochAngle%360));
            comDbg.debugMessage( "Sensing Left Wall dist: "+Double.toString(robot.getLeftDistance()));
            telemetry.addData("Sensing Left Wall dist: ", robot.getLeftDistance());
            telemetry.update();
            //drive(DS, robot.getHeadingGyro(), 28);
        }
        if(pos.equals("center"))
        {
            comDbg.debugMessage("Move back 5 inches & rotate 80.");
            driveback(DS, robot.getHeadingGyro(), 6);
            rotate(45 -180); //was 60 -180
            comDbg.debugMessage("C epochAngle after knocking & turning: " +Double.toString(epochAngle%360));
            //drive(DS, robot.getHeadingGyro(), 50);
            alignToWall("right", comDbg);
            strafeRight(0.5,4, comDbg);
            FollowAlongWall(.7, 40, 5,"right", "back", comDbg);
        }
        if(pos.equals("right")) {
            comDbg.debugMessage("Moving back 12 inches & rotate 102.");
            driveback(DS, robot.getHeadingGyro(), 12);
            rotate(-85); //was 75 -180
            comDbg.debugMessage("R epochAngle after knocking & turning: " +Double.toString(epochAngle%360));
            alignToWall("right", comDbg);
            strafeRight(0.5,4, comDbg);
            FollowAlongWall(.7,  42, 5,"right", "back", comDbg);

            //drive(DS, robot.getHeadingGyro(), 56);
        }
        //rotate((int) epochAngle - 45);
    }

    public void sampleGoldCrater(String pos, Debugger d)
    {
        globalAngle = 0;
        lastAngles = new Orientation(); //reset both just in case
        if (pos.equals("")) {
            pos = "center";
            d.debugMessage("DETECTION FAILED: FAKING GOLD POS AS CENTER.");
            telemetry.addData("FAILED DETECTION", pos);
            telemetry.update();
            telemetry.addData("Gold position", pos);
            telemetry.update();
        }

        if(pos.equals("left")) {
            rotate(33);
            if (epochAngle >= 34.0)
            {
                rotate(-1);
            }
            //d.debugMessage("Gold is on the left; rotating 33 deg left");
        }
        if(pos.equals("center"))
        {
            //d.debugMessage("Gold is in the center; not rotating");
        }
        if(pos.equals("right")) {
            rotate(-30);

            //d.debugMessage("Gold is on the right; rotating 30 deg right");
        }
        comDbg.debugMessage("epochAngle before knocking: " +Double.toString(epochAngle%360));

        drive(DS, robot.getHeadingGyro(), 21);// push the mineral
        //d.debugMessage("Moving forward 21 inches to hit mineral.");
        sleep(200);

        //d.debugMessage("Moving back 5 inches.");

        // position the robot to perpendicular to lander, face the wall at 45
        if(pos.equals("left")) {
            driveback(DS, robot.getHeadingGyro(), 4); // get the block out of the way
            rotate((int)(60.0 - epochAngle));
            comDbg.debugMessage("L epochAngle after knocking & turning: " +Double.toString(epochAngle%360));
            drive(DS, robot.getHeadingGyro(), 24);// 30 on encoder measured
        }
        if(pos.equals("center"))
        {
            d.debugMessage("Move back 5 inches & rotate 80.");
            driveback(DS, robot.getHeadingGyro(), 6);
            rotate(75);
            comDbg.debugMessage("C epochAngle after knocking & turning: " +Double.toString(epochAngle%360));
            drive(DS, robot.getHeadingGyro(), 45); // 51.5 on encoder measured; keep 4" away
        }
        if(pos.equals("right")) {
            d.debugMessage("Moving back 12 inches & rotate 102.");
            driveback(DS, robot.getHeadingGyro(), 12);
            rotate(102);
            comDbg.debugMessage("R epochAngle after knocking & turning: " +Double.toString(epochAngle%360));
            drive(DS, robot.getHeadingGyro(), 49); // 53.5 on encoder measured
        }

        //double encoderVal = robot.fr.getCurrentPosition();
        //while (robot.getFrontDistance() > 29) {
          //  robot.driveLimitless(-DS, DS, -DS, DS);
        //}
        //robot.allStop();
        //encoderVal -= robot.fr.getCurrentPosition();
        //comDbg.debugMessage(" Distance from lander to wall: " +Double.toString(encoderVal) +Double.toString(encoderVal/ENC_PER_INCH) );
        //sleep(100);
    }
    public void sampleGoldCraterNoSensing(String pos, Debugger d)
    {
        globalAngle = 0;
        lastAngles = new Orientation(); //reset both just in case
        if (pos.equals("")) {
            pos = "center";
            d.debugMessage("DETECTION FAILED: FAKING GOLD POS AS CENTER.");
            telemetry.addData("FAILED DETECTION", pos);
            telemetry.update();
            telemetry.addData("Gold position", pos);
            telemetry.update();
        }

        if(pos.equals("left")) {
            rotate(33);
            d.debugMessage("Gold is on the left; rotating 33 deg left");
        }
        if(pos.equals("center"))
        {
            d.debugMessage("Gold is in the center; not rotating");
        }
        if(pos.equals("right")) {
            rotate(-30);
            d.debugMessage("Gold is on the right; rotating 35 deg right");
        }
        comDbg.debugMessage("epochAngle before knocking: " +Double.toString(epochAngle%360));

        drive(DS, robot.getHeadingGyro(), 21);// push the mineral
        d.debugMessage("Moving forward 16 inches to hit mineral.");
        sleep(200);

        d.debugMessage("Moving back 5 inches.");

        // position the robot to perpendicular to lander, face the wall at 45
        if(pos.equals("left")) {
            driveback(DS, robot.getHeadingGyro(), 4); // get the block out of the way
            rotate(40);
            comDbg.debugMessage("L epochAngle after knocking & turning: " +Double.toString(epochAngle%360));
            drive(DS, robot.getHeadingGyro(), 28);

        }
        if(pos.equals("center"))
        {
            d.debugMessage("Move back 5 inches & rotate 80.");
            driveback(DS, robot.getHeadingGyro(), 6);
            rotate(80);
            comDbg.debugMessage("C epochAngle after knocking & turning: " +Double.toString(epochAngle%360));
            drive(DS, robot.getHeadingGyro(), 50);

        }
        if(pos.equals("right")) {
            d.debugMessage("Moving back 12 inches & rotate 102.");
            driveback(DS, robot.getHeadingGyro(), 12);
            rotate(102);
            comDbg.debugMessage("R epochAngle after knocking & turning: " +Double.toString(epochAngle%360));
            drive(DS, robot.getHeadingGyro(), 56);
        }

        //while (robot.getFrontDistance() > 29)
        //    robot.driveLimitless(-0.6, 0.6, -0.6, 0.6);
        stopRobot();
    }

    public void sampleGoldCraterWXtender(String pos, Debugger d)
    {
        globalAngle = 0;
        lastAngles = new Orientation(); //reset both just in case
        //pos = getGoldPosition();
        if (pos.equals("")) {
            pos = "center";
            telemetry.addData("FAILED DETECTION", pos);
            telemetry.update();
        }

        telemetry.addData("Gold position", pos);
        telemetry.update();

        if(pos.equals("left")) {
            rotate(23); //used to be 38
        }
        if(pos.equals("center"))
        {
        }
        if(pos.equals("right")) {
            rotate(-41);
        }

        robot.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && (robot.extender.getCurrentPosition() < (ENC_PER_INCH*12)))
        {
            robot.extender.setPower(0.9);
        }
        robot.extender.setPower(0);
        sleep(1000);

        robot.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive() && (robot.extender.getCurrentPosition() > -(ENC_PER_INCH*12)))
        {
            robot.extender.setPower(-0.9);
        }
        robot.extender.setPower(0);

// Revert back
        if(pos.equals("left")) {
            rotate(-38);
        }
        if(pos.equals("right")) {
            rotate(41);
        }
    }
    public String positionEstimateNoReInit(Debugger d){
        String pos1 = "", pos2 = "", pos3 = "", finalPos = "";
        waitForStart();
        pos1 = "left";//getGoldPosition(d);
        d.debugMessage("Gold position without flash is: " + pos1);
        CameraDevice.getInstance().setFlashTorchMode(true);
        pos2 = "right";//getGoldPosition(d);
        d.debugMessage("Gold position with flash is: " + pos2);
        telemetry.addData("Poses:",pos1+" "+pos2);
        telemetry.addData("Getting gold position is done", "press start");
        telemetry.update();


        landXplorer();
        if(pos1.equals(pos2)) //if first two positions agree, use them as final
        {
            finalPos = pos1;
            d.debugMessage("Pos 1 and 2 are equal; setting final pos to: " + pos1);
        }
        else {
            pos3 = getGoldPosition(d);
            d.debugMessage("Pos 1 and 2 are different; gold position from ground is: " + pos3);
            if(pos3.equals(pos1)) // if these two agree
            {
                finalPos = pos3;
                d.debugMessage("Gold position from ground and without flash agree; setting final pos to: " + pos3);
            }
            else if (pos3.equals(pos2)) //if these two agree
            {
                finalPos = pos2;
                d.debugMessage("Gold position from ground and with flash agree; setting final pos to: " + pos2);
            }
            else //nothing agrees
            {
                finalPos = pos3;
                d.debugMessage("Nothing agrees; setting final pos to: " + pos3);
            }
        }
        return finalPos;
    }
    public void grabGold()
    {
        robot.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //extend
        robot.extender.setPower(0.9);
        while(opModeIsActive() && (robot.extender.getCurrentPosition() < (ENC_PER_INCH*10)))
        {
        }
        //robot.extender.setPower(0);
        //grab
        double timeStart = System.currentTimeMillis();
        while ((opModeIsActive() && (System.currentTimeMillis() - timeStart < 1400)))
        {
            robot.intake.setPosition(0.12);
            robot.collector.setPower(1);
        }
        robot.extender.setPower(0);

        sleep(200);
        robot.intake.setPosition(0.4);
        robot.collector.setPower(0);
        sleep(200);
        //retract
        robot.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.extender.setPower(-0.9);
        while(opModeIsActive() && (robot.extender.getCurrentPosition()> -(ENC_PER_INCH*20)))
        {
        }
        robot.extender.setPower(0);
        sleep(200);
    }

    public void backToPark(Debugger d)
    {
        //driveback(DS, robot.getHeadingGyro(), 72);
        //driveback(0.4, robot.getHeadingGyro(), 4);
        resetAngle();
        //strafeAlongWall(0.70, 62,  4,  "right",  "back", d);//30 too short; 68 too far
        FollowAlongWall(0.70, 60,  4,  "right",  "back", d);//30 too short; 68 too far
        //60 almost works for depot center
        //robot.intake.setPosition(0.3);
        //sleep(300);
        //robot.loader.setPosition(0.3);
        //sleep(1000);
    }
    public void backToPark(Debugger d, String position)
    {
        //driveback(DS, robot.getHeadingGyro(), 72);
        //driveback(0.4, robot.getHeadingGyro(), 4);
        resetAngle();
        if (position.equals("right")) {
            strafeAlongWall(0.70, 60, 4, "right", "back", d);//30 too short; 68 too far
        }
        else if (position.equals("left"))
        {
            strafeAlongWall(0.70, 65, 4, "right", "back", d);//30 too short; 68 too far
        }
        //60 almost works for depot center
        //robot.intake.setPosition(0.3);
        //sleep(300);
        //robot.loader.setPosition(0.3);
        //sleep(1000);
    }
    public void depotToCrater(String pos, Debugger d)
    {
        while(opModeIsActive() && (robot.bl.getCurrentPosition() < (ENC_PER_INCH*3)))
        {
            robot.driveLimitless(DS, -DS, DS, -DS); //move back 3 inches
        }
        robot.markerMover.setPosition(0.15);
        if (pos.equals("right")) {
            //turn right by 90
            rotate(80);
        }
        else if (pos.equals("center"))
        {
            // turn right by 45
            rotate(73);
        }
        else if(pos.equals("left"))
        {
            //turn left by 90
            rotate( 80);
        }
        if (opModeIsActive()) {
            //strafeAlongWall(0.7, 16, 5, "right", "forward", d);
            FollowAlongWall(0.7, 8, 10, "right", "forward", d);
            stopRobot();

            strafeRight(0.5, 4, d);
            int dist = 56;
            if (pos.equals("left") )
            {
                dist -= 5;
            }
            if (pos.equals("right"))
            {
                dist -= 5;
            }

            FollowAlongWall(0.7, dist, 4, "right", "forward", d);
            stopRobot();
            //robot.intake.setPosition(0.44);
            //stopRobot();
        }
    }

    public void craterTurnToDepot(Debugger d)
    {
        //rotate(78);
        //drive(DS, robot.getHeadingGyro(), 54);
        //stopRobot();
        comDbg.debugMessage("epochAngle before Wall Turn: " +Double.toString(epochAngle%360));

        rotate((int)(90.0 - (epochAngle%360)) + 10); //previously 36 degrees; added a +10 correction
        stopRobot();
        //sleep(300);
        comDbg.debugMessage("epochAngle after Wall Turn: " +Double.toString(epochAngle%360));

        //drive(0.6, robot.getHeadingGyro(), 1); //used to be 4
        //comDbg.debugMessage("Drove 5 cm - no sensors.");
        //strafeAlongWallNoForwardSensor(0.60, 6,  4,  "right",  "forward", d);
        //comDbg.debugMessage("Strafed along wall with no forward sensor.");
        //strafeAlongWall(0.60, 19,  5,  "right",  "forward", d);
        strafeRight(0.5, 4,d);
        FollowAlongWall(0.65, 34,  4,  "right",  "forward", d);

        //comDbg.debugMessage("Strafed along wall with both sensors.");
        dropMarker();
        comDbg.debugMessage("Dropped marker.");
        sleep(400);

        stopRobot();
        comDbg.debugMessage("epochAngle after dropping: " +Double.toString(epochAngle%360));
    }

    public void craterTurnToPark()
    {
        stopRobot();
        rotate(-120); // turn to wall follow on right
        stopRobot();
    }

    public void driveUp(double dist) {
        drive(0.8, robot.getHeadingGyro(), dist);
    }
}
