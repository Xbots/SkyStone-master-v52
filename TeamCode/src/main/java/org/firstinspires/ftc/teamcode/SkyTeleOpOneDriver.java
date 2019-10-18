package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;


@TeleOp(name="Skystone: TeleOp Flex", group="Skystone")
public class SkyTeleOpOneDriver extends OpMode {

//
    HardwareStone robot = new HardwareStone();
    boolean closed = false;
    int close_count = 0;
    private double speed_multiplier = 0.8;
    private double speed_reverse = 1;                     // reverse the drive wheels
    private double  intakeOffset  = 0.0 ;                  // Servo mid position
    private double  markerOffset  = 0.0 ;                  // Servo mid position
    final double    INTAKE_SPEED  = 0.1 ;                 // sets rate to move servo
    Boolean SingleControl = true;
    double encVal =0;
    double loaderOffset = 0;
    public Gamepad Driver;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        
        if (SingleControl)
            Driver = gamepad1;
        else
            Driver = gamepad2;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Hardhat");    //
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


        if(Driver.left_stick_y == 0 && Driver.left_stick_x == 0 && !(abs(Driver.right_stick_x) > threshold))
        {
            robot.fr.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.bl.setPower(0);
        }
        else
        {
            robot.fl.setPower(speed_multiplier *( speed_reverse* (Driver.left_stick_y - Driver.left_stick_x)/2-(Driver.right_stick_x)/2));
            robot.bl.setPower(speed_multiplier *( speed_reverse* (Driver.left_stick_y + Driver.left_stick_x)/2-(Driver.right_stick_x)/2));
            robot.fr.setPower(speed_multiplier *( speed_reverse* (-Driver.left_stick_y - Driver.left_stick_x)/2-(Driver.right_stick_x)/2));
            robot.br.setPower(speed_multiplier *( speed_reverse* (-Driver.left_stick_y + Driver.left_stick_x)/2-(Driver.right_stick_x)/2));
        }



        // COLLECTION and INTAKE of MINERALS GAMEPAD1
        if(gamepad1.right_trigger >= 0.5) {
            robot.base.setPosition(0.12);
        }
        else
        if(gamepad1.left_trigger >= 0.5) {
            robot.base.setPosition(0.28);
        }


        /*
        // Use gamepad left & right Bumpers to move the intake
        if (gamepad1.right_bumper) {
            double timeStart = System.currentTimeMillis();

            robot.intake.setPosition(0.75);
        }
        */

        // LOAD MINERALS INTO CARGO HOLDER ON GAMEPAD1
        if(gamepad1.y) {
            robot.fl.setPower(1);
        }
        if(gamepad1.a)
            robot.fr.setPower(1);
        if (gamepad1.x)
            robot.bl.setPower(1);
        if (gamepad1.b)
            robot.bl.setPower(1);
        robot.fr.setPower(0);
        robot.fl.setPower(0);
        robot.br.setPower(0);
        robot.bl.setPower(0);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
    private void lift(boolean extend, int encCount)
    {
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (extend) {
            robot.lifter.setPower(-0.9);
            while ( robot.lifter.getCurrentPosition() > -encCount) {
            }
            ;
        }
        else {
            robot.lifter.setPower(0.9);
            while (robot.lifter.getCurrentPosition() < encCount) {
            }
            ;
        }
        robot.lifter.setPower(0);
    }
    */
}
