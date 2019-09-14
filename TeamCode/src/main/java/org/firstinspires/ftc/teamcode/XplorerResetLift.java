package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

@Autonomous(name = "Xplorer: Reset Lift", group = "Xplorer")
public class XplorerResetLift extends XplorerCommon {
  @Override
  public void runOpMode() {

      robot.init(hardwareMap);
      robot.markerMover.setPosition(0.57);
      robot.intake.setPosition(0.8);
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
      waitForStart();
      //robot.init(hardwareMap);
      lowerLift();
    }



}
