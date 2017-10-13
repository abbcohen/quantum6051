
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name="Omni with servos", group="Iterative Opmode")
public class OmniBaseCode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WheelOne = null;
    private DcMotor WheelTwo = null;
    private DcMotor WheelThree = null;
    private DcMotor WheelZero = null;
    private CRServo ServoFour = null;
    private CRServo ServoFive = null;
    private double moveSpeed = .75;
    private double turnSpeed = .5;
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
        //Servo initialization
        ServoFour = hardwareMap.get(CRServo.class, "ServoFour");
        ServoFive = hardwareMap.get(CRServo.class, "ServoFive");
        ServoFour.setDirection(CRServo.Direction.REVERSE);
        ServoFive.setDirection(CRServo.Direction.FORWARD);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void moveForward() {
        WheelOne.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelTwo.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelThree.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelZero.setPower(gamepad1.left_stick_y*moveSpeed);
    }
    public void moveBackward() {
        WheelOne.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelTwo.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelThree.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelZero.setPower(gamepad1.left_stick_y*moveSpeed);
    }
    public void moveLeft() {
        WheelOne.setPower(-gamepad1.left_stick_x*moveSpeed);
        WheelTwo.setPower(-gamepad1.left_stick_x*moveSpeed);
        WheelThree.setPower(gamepad1.left_stick_x*moveSpeed);
        WheelZero.setPower(gamepad1.left_stick_x*moveSpeed);
    }
    public void moveRight() {
        WheelOne.setPower(-gamepad1.left_stick_x*moveSpeed);
        WheelTwo.setPower(-gamepad1.left_stick_x*moveSpeed);
        WheelThree.setPower(gamepad1.left_stick_x*moveSpeed);
        WheelZero.setPower(gamepad1.left_stick_x*moveSpeed);
    }
    public void turnClockwise() {
        WheelOne.setPower(gamepad1.right_stick_x*turnSpeed);
        WheelTwo.setPower(-gamepad1.right_stick_x*turnSpeed);
        WheelThree.setPower(-gamepad1.right_stick_x*turnSpeed);
        WheelZero.setPower(gamepad1.right_stick_x*turnSpeed);
    }
    public void turnCounterClockwise() {
        WheelOne.setPower(gamepad1.right_stick_x*turnSpeed);
        WheelTwo.setPower(-gamepad1.right_stick_x*turnSpeed);
        WheelThree.setPower(-gamepad1.right_stick_x*turnSpeed);
        WheelZero.setPower(gamepad1.right_stick_x*turnSpeed);
    }
    public void driveStop(){
        WheelOne.setPower(0);
        WheelTwo.setPower(0);
        WheelThree.setPower(0);
        WheelZero.setPower(0);
    }
    public void grabberIn(){
        ServoFour.setPower(-gamepad1.right_trigger);
        ServoFive.setPower(-gamepad1.right_trigger);
    }
    public void grabberOut(){
        ServoFour.setPower(gamepad1.left_trigger);
        ServoFive.setPower(gamepad1.left_trigger);
    }
    public void grabberStop(){
        ServoFour.setPower(0);
        ServoFive.setPower(0);
    }


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
        //stopping
        if (gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0 && gamepad1.right_stick_x==0) driveStop();

       //moving forward/backward
        if (gamepad1.left_stick_y < -1/(10*moveSpeed)) moveForward();
        else if (gamepad1.left_stick_y > 1/(10*moveSpeed)) moveBackward();

        //moving left/right
        if (gamepad1.left_stick_x < -1/(10*moveSpeed)) moveLeft();
        else if (gamepad1.left_stick_x > 1/(10*moveSpeed)) moveRight();

        //turning
        if (gamepad1.right_stick_x < -1/(10*turnSpeed)) turnCounterClockwise();
        else if (gamepad1.right_stick_x > 1/(10*turnSpeed))turnClockwise();


        //grabber
        if (gamepad1.right_trigger > .1)grabberIn();
        else if (gamepad1.left_trigger > .1)grabberOut();
        else grabberStop();


        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("x_button", gamepad1.x);
        telemetry.addData("b_button", gamepad1.b);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}