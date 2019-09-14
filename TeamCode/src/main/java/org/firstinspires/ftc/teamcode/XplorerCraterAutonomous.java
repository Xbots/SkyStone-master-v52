package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Xplorer: Crater for States", group = "Xplorer")
public class XplorerCraterAutonomous extends XplorerCommon {

    VuforiaLocalizer vuforia;

    //HardwareXplorer robot = new HardwareXplorer();
    HardwareRover robot = new HardwareRover();


    double offsetGyro = 0.0;
    double startHeading = 0.0;
    final double GYRO_THRESHOLD = 0.5;
    final double CORRECTION_MULTIPLIER = 0.02;
    final double ENC_PER_INCH = 44;
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
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private final double ANGLE_THRESHOLD = 0.5;
    private String finalPos = "";
    private Debugger d = new Debugger("Crater debug");
  @Override
  public void runOpMode() {

      DateFormat date = new SimpleDateFormat("MM-dd-yyyy HH:mm:ss");
      d.openDebugFile("CRATER RUN: " + date.format(Calendar.getInstance().getTime()));
      initXplorer();
      finalPos = positionEstimateNoReInit(d);//positionEstimate(d); //samples twice in init, lands, then samples once if needed
      if (finalPos.equals("")) {
          d.setMsgPrefix("ERROR");
          d.debugMessage("Could not get gold position -  had to set to center");
          d.setMsgPrefix("INFO");
          finalPos = "center";
          telemetry.addData("Detected:", "none");
      } else {
          d.debugMessage("Gold position is " + finalPos + ".");
      }
      telemetry.addData("Gold  position:", finalPos);
      telemetry.update();

      if (opModeIsActive()) {


          escapeLatch();
          sampleGoldCrater(finalPos, d);  //Samples and ready to drop Team Marker
          d.debugMessage("Sampling done.");
          stopRobot();

          craterTurnToDepot(d);

          boolean doubleSampling = false;
          if (doubleSampling) {
              rotate(120);
              String doubleSamplePos = "";
              switch (finalPos) {
                  case "left":
                      doubleSamplePos = "right";
                      break;
                  case "right":
                      doubleSamplePos = "left";
                      break;
                  case "center":
                      doubleSamplePos = "center";
                      break;
              }
              justSample(doubleSamplePos, d);
              //craterTurnToPark();
          } else
              backToPark(d);
          d.closeDebugger();
      }
  }

}
