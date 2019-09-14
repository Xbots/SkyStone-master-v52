package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CameraDevice;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;

@Autonomous(name = "Xplorer: Diagnostic", group = "Xplorer")
public class XplorerDiagnostic extends XplorerCommon {
    VuforiaLocalizer vuforia;

    //HardwareXplorer robot = new HardwareXplorer();

    double offsetGyro = 0.0;
    double startHeading = 0.0;
    final double GYRO_THRESHOLD = 0.5;
    final double CORRECTION_MULTIPLIER = 0.04;
    final double ANGLE_THRESHOLD = 0.5;
    final double ENC_PER_INCH = 44;
    double forwardSpeed = 0.15;
    double leftChange, rightChange;
    double changeNum;
    double globalAngle, power = .30, correction;
        Orientation lastAngles = new Orientation();

    final double TURN_CORRECTION = 0.001;
    double turnSpeed = 0.5;
    double MIN_TURN_LIMIT = 0.1;
    private static final String VUFORIA_KEY = "AYEB3rP/////AAABmVMCZTLJIE0TrkP2639XOkl5oPNywXyUOnq52N57nxQ2Q4KVO6xRk1CWWvTIbeZfVku0ISp4m3dPUpfORFAHlDqKOCdLUfRP78YbJexyDYJ+q2KQlap1/SH5sZi/llSpn5Y0b30k/VK/txgdo7TsyZBNZrcldc0KRiwo3NVeQwuVHjxFLJsU0P3MmwNDKZ5Fax3l1yglpGB5Ej2Vevu1gmVfGxxcnMaw0m89+olVLW/rKOB1mOjNC4nwOMsljk/5uY0OMBfkm14r6/HnvXk5bX/GLyBL2gPRdmIL5wtAn0JDjoa+RkhA930mTv0h4Mzh1gZ3PLwWf3Nnt3T5Qu7ffT/RX9zPJ7pKOlp9m4a0Hfs5";
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private Debugger d = new Debugger("Diagnostic debug");
    @Override
    public void runOpMode() {
        DateFormat df = new SimpleDateFormat("MM-dd-yyyy HH:mm:ss");
        d.openDebugFile("DIAG RUN: " + df.format(Calendar.getInstance().getTime()));
        initXplorer();
        while(opModeIsActive())
        {
            d.debugMessage("Left distance: "+robot.getLeftDistance());
            d.debugMessage("Front Right distance: " + robot.getFrontRightDistance());
            d.debugMessage("Gyro heading: "+robot.getHeadingGyro());
            d.debugMessage("Lifter encoder: " +robot.lifter.getCurrentPosition());
            com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);
            String pos = getGoldPosition(d);
            d.debugMessage("Current gold position: " + pos);
            telemetry.addData("Gold position:", pos);
            telemetry.update();
        }
    }
}
