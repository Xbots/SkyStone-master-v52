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
        telemetry.addData("Say", "Hello Slow Bilda!");    //
        updateTelemetry(telemetry);
    }

    public void init_loop() {
        telemetry.addData("Init", " Slow Bilda!");    //
        updateTelemetry(telemetry);
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

        robot.resetEncoders();

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

            telemetry.addData("fl.enc ",robot.fl.getCurrentPosition());
            telemetry.addData("fr.enc ",robot.fr.getCurrentPosition());
            telemetry.addData("bl.enc ",robot.bl.getCurrentPosition());
            telemetry.addData("br.enc ",robot.br.getCurrentPosition());
            telemetry.update();
        }


    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
