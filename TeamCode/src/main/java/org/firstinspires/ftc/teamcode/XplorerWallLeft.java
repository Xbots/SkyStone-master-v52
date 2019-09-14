package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

@Autonomous(name = "Xplorer: Wall Follow Left", group = "Xplorer")
public class XplorerWallLeft extends XplorerCommon {

    VuforiaLocalizer vuforia;

    HardwareXplorer robot = new HardwareXplorer();

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
    private Debugger d = new Debugger("Wall debug");
    String pos;

  @Override
  public void runOpMode() {

      initXplorer();
      waitForStart();
      DateFormat date = new SimpleDateFormat("MM-dd-yyyy HH:,mm:ss");
      d.openDebugFile("WALL RUN: " + date.format(Calendar.getInstance().getTime()));
      //straightenToWall("right", d);
      alignToWall("left", d);
      strafeLeft(0.5, 4, d);
      //sleep(2000);
      //strafeAlongWall(0.3, 24, 3, "left", "forward", d);
      stopRobot();
      //travelAlongWall(0.3, 54,  4,  "right",  "forward", d);
      //sleep(5000);
      //strafeAlongWall(0.7, 15,  3,  "right",  "forward", d);
  }
}
