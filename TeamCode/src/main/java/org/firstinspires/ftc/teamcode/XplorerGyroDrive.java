package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;


public class XplorerGyroDrive extends LinearOpMode{


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

    private final double ANGLE_THRESHOLD = 0.5;


    @Override
    public void runOpMode()
    {
        return;
    }
    public void initXplorer()
    {
        robot.init(hardwareMap);

        offsetGyro = robot.getHeadingGyro();
        startHeading = robot.getHeadingGyro()-offsetGyro;

        globalAngle = 0;
        lastAngles = new Orientation(); //reset both just in case

        robot.resetEncoders();

        telemetry.addData("Init", "done");
        telemetry.update();
    }


    public void stopRobot()
    {
        robot.allStop();
        robot.resetEncoders();
        sleep(60);
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
    public void straightenToWall(String wallSide, Debugger d)
    {
        double frontSum = 0, backSum = 0;


        if (wallSide == "left")
            return;

        double x = robot.getFrontRightDistance();
        double y = robot.getBackRightDistance();

        if (x >0 && y > 0 && x < 16 && y < 16) {
            for (int i = 0; i < 5; i++) {
                frontSum += robot.getFrontRightDistance();
                backSum += robot.getBackRightDistance();
            }
            if (frontSum / 5 > 100 || backSum / 5 > 100) {
                d.debugMessage("One of the distance sensors is not working. Unable to straighten robot.");
            } else {
                if (frontSum > backSum) // front of robot is further away; need to turn right
                    // using diff powers to right and left wheels rather than rotate
                    while (Math.abs(robot.getFrontRightDistance() - robot.getBackRightDistance()) > 1) { //was 4
                        robot.driveLimitless(-0.7, 0.5, -0.7, 0.5);  // left 0.7 right 0.5
                        //rotate(-6);
                        d.debugMessage("Front right distance: " + robot.getFrontRightDistance());
                        d.debugMessage("Back right distance: " + robot.getBackRightDistance());
                    }

                else if (frontSum < backSum) // back of robot is further away; need to turn left
                    while (Math.abs(robot.getFrontRightDistance() - robot.getBackRightDistance()) > 1) { // was 4
                        // using diff powers to right and left wheels rather than rotate
                        robot.driveLimitless(-0.5, 0.7, -0.5, 0.7);  // left 0.5 right 0.7
                        //rotate(6);
                        d.debugMessage("Front right distance: " + robot.getFrontRightDistance());
                        d.debugMessage("Back right distance: " + robot.getBackRightDistance());
                    }
            }
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
    public void strafeAlongWall(double speed, double distToTravel, double distFromWall, String wallSide, String direction, Debugger d) {
        distToTravel = distToTravel * ENC_PER_INCH;

        robot.resetEncoders();
        telemetry.addData("moving forward limitlessly...", "");
        telemetry.update();
        straightenToWall(wallSide, d);
        while(opModeIsActive()) {
            while ( ((robot.getFrontDistance() > 21 && direction.equals("forward")) //13 gets caught sometimes
                    || (robot.fl.getCurrentPosition() < distToTravel && direction.equals("back")))) {
                wallStrafe(speed, distFromWall, wallSide, direction, d);
            }
            double x = robot.getFrontDistance();
            if (x <= 18 && direction.equals("forward")) {
                telemetry.addData("critical: ", x);
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



    public void driveUp(double dist) {
        drive(0.8, robot.getHeadingGyro(), dist);
    }
}
