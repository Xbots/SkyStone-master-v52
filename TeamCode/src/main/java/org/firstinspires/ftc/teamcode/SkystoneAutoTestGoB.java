package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

@Autonomous(name = "GoBilda: AutoTest", group = "Skystone")
public class SkystoneAutoTestGoB extends XplorerCommon {

    VuforiaLocalizer vuforia;

    //HardwareXplorer robot = new HardwareXplorer();
    HardwareGobilda robot = new HardwareGobilda();


    double offsetGyro = 0.0;
    double startHeading = 0.0;
    final double GYRO_THRESHOLD = 0.5;
    final double CORRECTION_MULTIPLIER = 0.02;
    final double ENC_PER_INCH = 31; // relibrated for GoBilda 5202; AndyMark was 44
    double forwardSpeed = 0.15;
    double leftChange, rightChange;
    double changeNum;
    final double DRIVE_SPEED = 0.1; //was 0.7
    final double DS = 0.7;
    final double MAX_SPEED = 1;
    final double TURN_CORRECTION = 0.001;
    double turnSpeed = 0.5;
    double MIN_TURN_LIMIT = 0.1;
    double globalAngle, power = .30, correction;
    Orientation lastAngles = new Orientation();
    private static final String VUFORIA_KEY = "AYEB3rP/////AAABmVMCZTLJIE0TrkP2639XOkl5oPNywXyUOnq52N57nxQ2Q4KVO6xRk1CWWvTIbeZfVku0ISp4m3dPUpfORFAHlDqKOCdLUfRP78YbJexyDYJ+q2KQlap1/SH5sZi/llSpn5Y0b30k/VK/txgdo7TsyZBNZrcldc0KRiwo3NVeQwuVHjxFLJsU0P3MmwNDKZ5Fax3l1yglpGB5Ej2Vevu1gmVfGxxcnMaw0m89+olVLW/rKOB1mOjNC4nwOMsljk/5uY0OMBfkm14r6/HnvXk5bX/GLyBL2gPRdmIL5wtAn0JDjoa+RkhA930mTv0h4Mzh1gZ3PLwWf3Nnt3T5Qu7ffT/RX9zPJ7pKOlp9m4a0Hfs5";
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private final double ANGLE_THRESHOLD = 0.5;
    private String finalPos = "";
    private Debugger d = new Debugger("Crater debug");
  @Override
  public void runOpMode() {

      robot.init(hardwareMap);

      DateFormat date = new SimpleDateFormat("MM-dd-yyyy HH:mm:ss");
      d.openDebugFile("CRATER RUN: " + date.format(Calendar.getInstance().getTime()));
      //finalPos = positionEstimateNoReInit(d);//positionEstimate(d); //samples twice in init, lands, then samples once if needed

    waitForStart();

      if (opModeIsActive()) {
          //sampleGoldCrater(finalPos, d);  //Samples and ready to drop Team Marker
          //d.debugMessage("Sampling done.");
          //stopRobot();
          driveX(DS, 20);
          d.closeDebugger();
      }
  }

    public void driveX(double forwardSpeed, double dist){
        robot.resetEncoders();
        double startHeading = robot.getHeadingGyro();
        dist = dist * ENC_PER_INCH;
        telemetry.addData("moving forward (enc val)...", dist);
        telemetry.update();

        double encVal = robot.fr.getCurrentPosition();
        telemetry.addData("enc val start: ", encVal);

        while(opModeIsActive()  && robot.fl.getCurrentPosition() < dist) {
            telemetry.addData("enc val: ", encVal);
            telemetry.update();
            moveForward (forwardSpeed, startHeading);
        }
        telemetry.update();
        robot.allStop();
    }

    public void driveXback(double backwardSpeed, double dist){
        backwardSpeed = Math.abs(backwardSpeed);
        robot.resetEncoders();
        //telemetry.addData("moving backward...", dist);
        //telemetry.update();
        dist = dist * ENC_PER_INCH;
        double startHeading = robot.getHeadingGyro();

        while(opModeIsActive()  && robot.fr.getCurrentPosition() > -dist) {
            moveBackward (backwardSpeed, startHeading);
        }
        robot.allStop();
    }
    
    public void moveForward(double forwardSpeed, double startHeading){
        leftChange = forwardSpeed;
        rightChange = forwardSpeed;
        if(robot.getHeadingGyro() - offsetGyro - startHeading > GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getHeadingGyro() - offsetGyro - startHeading);
            rightChange += CORRECTION_MULTIPLIER * changeNum;
            telemetry.addData("heading: ", robot.getHeadingGyro());
            telemetry.addData("right adj: ", rightChange);
        }else if(robot.getHeadingGyro() - offsetGyro - startHeading < -GYRO_THRESHOLD){
            changeNum = Math.abs(robot.getHeadingGyro() - offsetGyro - startHeading);
            leftChange += CORRECTION_MULTIPLIER * changeNum;
            telemetry.addData("heading: ", robot.getHeadingGyro());
            telemetry.addData("left adj: ", leftChange);
        }else{
            //robot is driving within acceptable range
        }
        robot.driveLimitless(-leftChange, rightChange, -leftChange, rightChange);
        telemetry.update();
        //comDbg.debugMessage("MvFwd: left Change, right Change: " + Double.toString(leftChange) +", " +Double.toString(rightChange));
    }
}
