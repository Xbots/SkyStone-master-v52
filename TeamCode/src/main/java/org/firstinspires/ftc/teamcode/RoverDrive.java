package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static android.os.SystemClock.sleep;


@TeleOp(name="Rover: Drive", group="Rover")
@Disabled
public class RoverDrive extends OpMode {


    HardwareRover robot = new HardwareRover();
    boolean closed = false;
    int close_count = 0;
    double speed_multiplier = 1;
    double speed_reverse = 1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        //debug
        telemetry.addData("count ", close_count);
        telemetry.update();


        if(gamepad1.dpad_up){
            speed_reverse = 1;
        }else if(gamepad1.dpad_down){
            speed_reverse = -1;
        }

        if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 && !(Math.abs(gamepad1.right_stick_x) > threshold))
        {
            robot.fr.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.bl.setPower(0);
        }
        else
        {
            robot.fl.setPower(speed_multiplier *( speed_reverse* (gamepad1.left_stick_y - gamepad1.left_stick_x)/2-(gamepad1.right_stick_x)/2));
            robot.bl.setPower(speed_multiplier *( speed_reverse* (gamepad1.left_stick_y + gamepad1.left_stick_x)/2-(gamepad1.right_stick_x)/2));
            robot.fr.setPower(speed_multiplier *( speed_reverse* (-gamepad1.left_stick_y - gamepad1.left_stick_x)/2-(gamepad1.right_stick_x)/2));
            robot.br.setPower(speed_multiplier *( speed_reverse* (-gamepad1.left_stick_y + gamepad1.left_stick_x)/2-(gamepad1.right_stick_x)/2));
        }


        if(gamepad1.y) {// forward
            robot.driveLimitless(1, -1, 1, -1);
        }
        else
        if(gamepad1.x) { // left
            robot.driveLimitless(1, 1, -1, -1);
        }
        else
        if(gamepad1.a) {//backwards
            robot.driveLimitless(-1, 1, -1, 1);
        }
        else
        if(gamepad1.b) { // right
            robot.driveLimitless(-1, -1, 1, 1);
        }
        else
            robot.allStop();
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
