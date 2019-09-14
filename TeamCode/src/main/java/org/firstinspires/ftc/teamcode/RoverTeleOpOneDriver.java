package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;


@TeleOp(name="Rover: TeleOp Flex", group="Rover")
public class RoverTeleOpOneDriver extends OpMode {

//
    HardwareRover robot = new HardwareRover();
    boolean closed = false;
    int close_count = 0;
    private double speed_multiplier = 0.8;
    private double speed_reverse = -1;                     // reverse the drive wheels
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

        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.intake.setPosition(0.6);
        robot.markerMover.setPosition(0.57);
        robot.loader.setPosition(1.0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Astronaut");    //
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
                &&!Driver.y &&!Driver.a) {
            robot.intake.setPosition(0.4);
        }

        if(gamepad1.dpad_up){
            robot.extender.setPower(1);
        }else if(gamepad1.dpad_down){
            robot.extender.setPower(-1);
        }
        robot.extender.setPower(0);

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


        double totalPulleyPower; //left is up right is down

        if(Driver.dpad_left) {
            totalPulleyPower = 0.9;
        }
        else if(Driver.dpad_right) {
            totalPulleyPower = -0.9;
        }
        else
            totalPulleyPower = 0;
        robot.lifter.setPower(totalPulleyPower);

        // COLLECTION and INTAKE of MINERALS GAMEPAD1
        if(gamepad1.right_trigger >= 0.5) {
            robot.intake.setPosition(0.12);
            collectorPower = 1;
        }
        else
        if(gamepad1.left_trigger >= 0.5) {
            robot.intake.setPosition(0.12);
            collectorPower = -1;
        }
        else {
            collectorPower = 0;
        }
        robot.collector.setPower(collectorPower);

        // Use gamepad left & right Bumpers to move the intake
        if (gamepad1.right_bumper) {
            double timeStart = System.currentTimeMillis();

            robot.intake.setPosition(0.75);
        }

        // LOAD MINERALS INTO CARGO HOLDER ON GAMEPAD1
        if(gamepad1.y) {
            // get rid of collection box in the way
            if (robot.intake.getPosition() > 0.4)
               robot.intake.setPosition(0.5);
            distPower = 1;
        }
        else if(gamepad1.a)
            distPower = -1;
        else
            distPower = 0;
        robot.distributor.setPower(distPower);

        if (gamepad1.x)
            loaderOffset += INTAKE_SPEED;
        else if (gamepad1.b)
            loaderOffset -= INTAKE_SPEED;

        if (gamepad1.x ) {
            robot.loader.setPosition(1);
        }
        if (gamepad1.b){
            loaderOffset = Range.clip(loaderOffset, -0.5, 0.5);
            robot.loader.setPosition(robot.MID_SERVO + loaderOffset);
        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
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
}
