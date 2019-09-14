package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Zane: Move Arm Down", group="Zane")
@Disabled
public class MoveArmDownZane extends OpMode {


    HardwareZane robot = new HardwareZane();
    boolean closed = false;
    int close_count = 0;
    double speed_multiplier = 1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    @Override
    public void loop() {

        double totalPowerLift; //left is up right is down
        if(gamepad1.y)
            totalPowerLift = 0.7;
        else if(gamepad1.a)
            totalPowerLift = -0.7;
        else
            totalPowerLift = 0;
        robot.lifter.setPower(totalPowerLift);


        double totalPowerExtend;
        totalPowerExtend = gamepad1.left_stick_y;
        robot.extender.setPower(totalPowerExtend);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("stopping", 0);
        telemetry.update();
    }

}
