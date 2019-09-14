package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Xplorer: Raise Lift", group = "Xplorer")
public class XplorerRaiseLift extends XplorerCommon {
  @Override
  public void runOpMode() {

      robot.init(hardwareMap);

      robot.resetEncoders();

      waitForStart();
      //robot.init(hardwareMap);
      raiseFull();
    }



}
