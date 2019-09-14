
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ForkBot: Move Down Arm", group="Apollo")
@Disabled
public class MoveArmDownForkBot extends OpMode {

    /* Declare OpMode members. */
    HardwareForkBot robot       = new HardwareForkBot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.

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

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*telemetry.addData("",robot.raiser.getCurrentPosition());
        boolean stopcommand = false;
        while(stopcommand = false)
        {
            robot.raiser.setPower(-0.4);
        }
        if(gamepad1.a)
        {
            stopcommand = true;
        }
        updateTelemetry(telemetry);
        */
        robot.raiser.setPower(0.4);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.raiser.setPower(0.0);
    }

}