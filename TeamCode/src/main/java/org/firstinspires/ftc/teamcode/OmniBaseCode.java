
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name="Emlyn Sucks (with a lift)", group="Iterative Opmode")
public class OmniBaseCode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WheelOne = null;
    private DcMotor WheelTwo = null;
    private DcMotor WheelThree = null;
    private DcMotor WheelZero = null;
    private DcMotor LiftMotor = null;
    private Servo GlyphServo1 = null;
    private Servo GlyphServo2 = null;
    private RevRoboticsCoreHexMotor GlyphWheel1 = null;
    private RevRoboticsCoreHexMotor GlyphWheel2 = null;
    private Servo JewelServo = null;
    private double moveSpeed = .75;
    private double turnSpeed = .5;
    private double liftSpeed = 1;
    private boolean slomo;
    public double servoPos = .5;
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
        GlyphWheel1 = hardwareMap.get(RevRoboticsCoreHexMotor.class, "GlyphWheel1");
        GlyphWheel2 = hardwareMap.get(RevRoboticsCoreHexMotor.class, "GlyphWheel2");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        WheelOne.setDirection(DcMotor.Direction.FORWARD);
        WheelTwo.setDirection(DcMotor.Direction.REVERSE);
        WheelThree.setDirection(DcMotor.Direction.REVERSE);
        WheelZero.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setDirection(DcMotor.Direction.REVERSE);
        //Servo initialization
        GlyphServo1 = hardwareMap.get(Servo.class, "GlyphServo1");
        GlyphServo1.setDirection(Servo.Direction.REVERSE);
        GlyphServo2 = hardwareMap.get(Servo.class, "GlyphServo2");
        GlyphServo2.setDirection(Servo.Direction.FORWARD);
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        JewelServo.setDirection(Servo.Direction.REVERSE);
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
    public void succ() {
        GlyphWheel1.equals(.6);
        GlyphWheel2.equals(.6);
    }
    public void nosucc() {
        GlyphWheel1.equals(0);
        GlyphWheel2.equals(0);
    }
    public void blow() {
        GlyphWheel1.equals(-.6);
        GlyphWheel2.equals(-.6);
    }
    public void grabber(double pos) {
            GlyphServo1.setPosition(pos);
            GlyphServo2.setPosition(pos);
    }
    public void jewel(char button){
        if(button == 'a') JewelServo.setPosition(90);
        if(button =='y') JewelServo.setPosition(0);
    }

    public void liftUp(){
        LiftMotor.setPower(gamepad2.right_stick_y*liftSpeed);
    }
    public void liftDown(){
        LiftMotor.setPower(gamepad2.right_stick_y*liftSpeed);
    }
    public void liftStop(){
        LiftMotor.setPower(0);
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
        //slow-motion code:
        if(gamepad1.right_bumper==true) slomo=!slomo;
        if(slomo){
            moveSpeed = .25;
        }
        else{
            moveSpeed = .75;
        }
        //stopping
        if (gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0 && gamepad1.right_stick_x==0) driveStop();

        //moving forward/backward
        if (gamepad1.left_stick_y < -.2) moveForward();
        else if (gamepad1.left_stick_y > .2) moveBackward();

        //moving left/right
        if (gamepad1.left_stick_x < -.2) moveLeft();
        else if (gamepad1.left_stick_x > .2) moveRight();

        //turning
        if (gamepad1.right_stick_x < -.2) turnCounterClockwise();
        else if (gamepad1.right_stick_x > .2)turnClockwise();

        //glyph wheels
        if(gamepad2.dpad_down) succ();
        else if(gamepad2.dpad_up) blow();
        else nosucc();


        //grabber
        if (gamepad2.right_bumper) grabber(95); //in
        else if (gamepad2.left_bumper) grabber(85); //lil out
        else if (gamepad2.left_trigger> 0.1) grabber(50);//far out

        //jewel
        if (gamepad2.a) JewelServo.setPosition(0);
        else if(gamepad2.y) JewelServo.setPosition(70);

        //lift
        if((gamepad2.right_stick_y > -.1) && !(gamepad2.right_stick_y<.1)) liftUp();
        else if (gamepad2.right_stick_y < .1 && !(gamepad2.right_stick_y>-.1))liftDown();
        else liftStop();

        telemetry.addData("Glyph Servo 1", GlyphServo1.getPosition());
        telemetry.addData("Glyph Servo 2", GlyphServo2.getPosition());
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("y_button", gamepad1.y);
        telemetry.addData("a_button", gamepad1.a);
        telemetry.addData("right bumper", gamepad2.right_bumper);
        telemetry.addData("left bumper", gamepad2.left_bumper);
        telemetry.addData("lift", gamepad2.right_stick_y);
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