package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;


@TeleOp(name="GoBilda: TeleOp Demo", group="GoBilda")
public class GobildaDemo extends OpMode {

//
    HardwareGobilda robot = new HardwareGobilda();
    boolean closed = false;
    int close_count = 0;
    private double speed_multiplier = 0.8;
    private double speed_reverse = 1;
    private double  intakeOffset  = 0.0 ;                  // Servo mid position
    private double  markerOffset  = 0.0 ;                  // Servo mid position
    final double    INTAKE_SPEED  = 0.1 ;                 // sets rate to move servo
    double encVal =0;
    double loaderOffset = 0;

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
        telemetry.addData("Say", "Hello GoBilda!");    //
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

        double threshold = 0.1;
        double collectorPower = 0;
        double distPower = 0;

        // Keep the collector box flat/horizontal position if these buttons are not pressed
        if(!gamepad1.dpad_up && !gamepad1.dpad_right &&!gamepad1.dpad_left && !gamepad1.y
                && !gamepad1.right_bumper  && (gamepad1.right_trigger==0) && (gamepad1.left_trigger==0)
                &&!gamepad2.y &&!gamepad2.a) {
            //robot.intake.setPosition(0.4);
            // buttons to trigger flatness
            // drive ( gamepad2.left/right_x/y;
            // g1.dpad_down ( extender retracts
            // g1.x ( loader lowers )
            // g1.a g1.b
            // g1.left_bumper
            // can remove code below in some sections if it works
            // main goal - to bring the collector back up flat once trigger is released - will it work?
        } 


        if(gamepad2.left_stick_y == 0 && gamepad2.left_stick_x == 0 && !(abs(gamepad2.right_stick_x) > threshold))
        {
            robot.fr.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.bl.setPower(0);
        }
        else
        {
            robot.fl.setPower(speed_multiplier *( speed_reverse* (gamepad2.left_stick_y - gamepad2.left_stick_x)/2-(gamepad2.right_stick_x)/2));
            robot.bl.setPower(speed_multiplier *( speed_reverse* (gamepad2.left_stick_y + gamepad2.left_stick_x)/2-(gamepad2.right_stick_x)/2));
            robot.fr.setPower(speed_multiplier *( speed_reverse* (-gamepad2.left_stick_y - gamepad2.left_stick_x)/2-(gamepad2.right_stick_x)/2));
            robot.br.setPower(speed_multiplier *( speed_reverse* (-gamepad2.left_stick_y + gamepad2.left_stick_x)/2-(gamepad2.right_stick_x)/2));
        }


    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
