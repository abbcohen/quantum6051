

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="OMNI BOI", group="Iterative Opmode")
public class OmniBaseCode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WheelOne = null;
    private DcMotor WheelTwo = null;
    private DcMotor WheelThree = null;
    private DcMotor WheelZero = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        WheelOne  = hardwareMap.get(DcMotor.class, "WheelOne");
        WheelTwo  = hardwareMap.get(DcMotor.class, "WheelTwo");
        WheelThree = hardwareMap.get(DcMotor.class, "WheelThree");
        WheelZero = hardwareMap.get(DcMotor.class, "WheelZero");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        WheelOne.setDirection(DcMotor.Direction.FORWARD);
        WheelTwo.setDirection(DcMotor.Direction.REVERSE);
        WheelThree.setDirection(DcMotor.Direction.REVERSE);
        WheelZero.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        WheelOne.setPower(0);
        WheelTwo.setPower(0);
        WheelThree.setPower(0);
        WheelZero.setPower(0);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


//MOVING (NOT TURNING) CODE
        //move forward
        if (gamepad1.left_stick_y < -.1) {
            WheelOne.setPower(gamepad1.left_stick_y);
            WheelTwo.setPower(gamepad1.left_stick_y);
            WheelThree.setPower(gamepad1.left_stick_y);
            WheelZero.setPower(gamepad1.left_stick_y);
        }

        //move backward
        else if (gamepad1.left_stick_y > .1) {
            WheelOne.setPower(gamepad1.left_stick_y);
            WheelTwo.setPower(gamepad1.left_stick_y);
            WheelThree.setPower(gamepad1.left_stick_y);
            WheelZero.setPower(gamepad1.left_stick_y);
        }
        //move left
        if (gamepad1.left_stick_x < -.1) {
            WheelOne.setPower(gamepad1.left_stick_x);
            WheelTwo.setPower(gamepad1.left_stick_x);
            WheelThree.setPower(-gamepad1.left_stick_x);
            WheelZero.setPower(-gamepad1.left_stick_x);
        }
        //move right
        else if (gamepad1.left_stick_x > .1) {
            telemetry.addLine("BEN CW");
            WheelOne.setPower(gamepad1.left_stick_x);
            WheelTwo.setPower(gamepad1.left_stick_x);
            WheelThree.setPower(-gamepad1.left_stick_x);
            WheelZero.setPower(-gamepad1.left_stick_x);
        }
        //nothing
        else{
            WheelOne.setPower(0);
            WheelTwo.setPower(0);
            WheelThree.setPower(0);
            WheelZero.setPower(0);
        }
        telemetry.addData("leftsticky", gamepad1.left_stick_y);
        telemetry.addData("leftstickx", gamepad1.left_stick_x);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);


        //turn left
        if (gamepad1.right_stick_x < -.1) {
            WheelOne.setPower(-gamepad1.left_stick_x);
            WheelTwo.setPower(-gamepad1.left_stick_x);
            WheelThree.setPower(-gamepad1.left_stick_x);
            WheelZero.setPower(-gamepad1.left_stick_x);
        }
        //turn right
        else if (gamepad1.right_stick_x > .1) {
            WheelOne.setPower(gamepad1.left_stick_x);
            WheelTwo.setPower(gamepad1.left_stick_x);
            WheelThree.setPower(gamepad1.left_stick_x);
            WheelZero.setPower(gamepad1.left_stick_x);
        }


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}