
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="succ, grab, lyft", group="Iterative Opmode")
public class OmniBaseCode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private DcMotor LiftMotor = null;
    private Servo GlyphServo1 = null;
    private Servo GlyphServo2 = null;
    private DcMotor GlyphWheel1 = null;
    private DcMotor GlyphWheel2 = null;
    private DcMotor GlyphWheel3 = null;
    private Servo PushServo = null;
    private double moveSpeed = .75;
    private double turnSpeed = .5;
    private double liftSpeed = 1;
    private boolean slomo;
    private boolean push = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        GlyphWheel1 = hardwareMap.get(DcMotor.class, "GlyphWheel1");
        GlyphWheel2 = hardwareMap.get(DcMotor.class, "GlyphWheel2");
        GlyphWheel3 = hardwareMap.get(DcMotor.class, "GlyphWheel3");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        GlyphWheel1.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheel2.setDirection(DcMotor.Direction.REVERSE);
        GlyphWheel3.setDirection(DcMotor.Direction.REVERSE);
        //Servo initialization
        GlyphServo1 = hardwareMap.get(Servo.class, "GlyphServo1");
        GlyphServo1.setDirection(Servo.Direction.REVERSE);
        GlyphServo2 = hardwareMap.get(Servo.class, "GlyphServo2");
        GlyphServo2.setDirection(Servo.Direction.FORWARD);
        PushServo = hardwareMap.get(Servo.class, "PushServo");
        PushServo.setDirection(Servo.Direction.REVERSE);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        /*GlyphServo1.scaleRange(0,180);
        GlyphServo2.scaleRange(0,180);
        GlyphServo1.setPosition(50);
        GlyphServo2.setPosition(50);
        */
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

//    public void moveForward() {
//        WheelOne.setPower(gamepad1.left_stick_y*moveSpeed);
//        WheelTwo.setPower(gamepad1.left_stick_y*moveSpeed);
//        WheelThree.setPower(gamepad1.left_stick_y*moveSpeed);
//        WheelZero.setPower(gamepad1.left_stick_y*moveSpeed);
//    }
//    public void moveBackward() {
//        WheelOne.setPower(gamepad1.left_stick_y*moveSpeed);
//        WheelTwo.setPower(gamepad1.left_stick_y*moveSpeed);
//        WheelThree.setPower(gamepad1.left_stick_y*moveSpeed);
//        WheelZero.setPower(gamepad1.left_stick_y*moveSpeed);
//    }
//    public void moveLeft() {
//        WheelOne.setPower(-gamepad1.left_stick_x*moveSpeed);
//        WheelTwo.setPower(-gamepad1.left_stick_x*moveSpeed);
//        WheelThree.setPower(gamepad1.left_stick_x*moveSpeed);
//        WheelZero.setPower(gamepad1.left_stick_x*moveSpeed);
//    }
//    public void moveRight() {
//        WheelOne.setPower(-gamepad1.left_stick_x*moveSpeed);
//        WheelTwo.setPower(-gamepad1.left_stick_x*moveSpeed);
//        WheelThree.setPower(gamepad1.left_stick_x*moveSpeed);
//        WheelZero.setPower(gamepad1.left_stick_x*moveSpeed);
//    }
//    public void turnClockwise() {
//        WheelOne.setPower(gamepad1.right_stick_x*turnSpeed);
//        WheelTwo.setPower(-gamepad1.right_stick_x*turnSpeed);
//        WheelThree.setPower(-gamepad1.right_stick_x*turnSpeed);
//        WheelZero.setPower(gamepad1.right_stick_x*turnSpeed);
//    }
//    public void turnCounterClockwise() {
//        WheelOne.setPower(gamepad1.right_stick_x*turnSpeed);
//        WheelTwo.setPower(-gamepad1.right_stick_x*turnSpeed);
//        WheelThree.setPower(-gamepad1.right_stick_x*turnSpeed);
//        WheelZero.setPower(gamepad1.right_stick_x*turnSpeed);
//    }

    public void driveStop(){
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
    public void succ (double speed) {
        GlyphWheel1.setPower(speed);
        GlyphWheel2.setPower(speed);
    }

//    public void jewel(char button){
//        if(button == 'a') JewelServo.setPosition(90);
//        if(button =='y') JewelServo.setPosition(0);
//    }
    public void push(){
        if (push) PushServo.setPosition(90);
        else PushServo.setPosition(15);
        push=!push;
    }

    public void liftUp(){
        LiftMotor.setPower(gamepad2.right_stick_y*liftSpeed);
    }
    public void liftDown(){
        LiftMotor.setPower(gamepad2.right_stick_y*liftSpeed);
    }
    public void liftNoPower(){
        LiftMotor.setPower(0);
    }
    public void liftStop(){
        LiftMotor.setPower(.05);
    }


    @Override
    public void init_loop() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

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

        //switch slomo
        if(gamepad1.right_bumper) slomo=true;
        else slomo=false;

        //set speed of slomo
        if (!slomo){
            moveSpeed = .75;
        }else if (slomo){
            moveSpeed = .6;
        }
        //stopping
        if (gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0 && gamepad1.right_stick_x==0) driveStop();

        //driving
        double scale = (gamepad1.right_bumper ? .3 : .7);
        double drive_scale = (gamepad1.right_bumper ? .3 : 1);

        double gamepad1LeftY = -gamepad1.left_stick_y * drive_scale;
        double gamepad1LeftX = gamepad1.left_stick_x * drive_scale;
        double gamepad1RightX = gamepad1.right_stick_x * scale;
        double frontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        double frontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        double backRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        double backLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        if (Math.abs(gamepad1LeftX) > .2 || Math.abs(gamepad1LeftY) > .2 || Math.abs(gamepad1RightX) > .2) {
            // holonomic formulas

            // clip the right/left values so that the values never exceed +/- 1
            frontRight = Range.clip(frontRight, -1, 1);
            frontLeft = Range.clip(frontLeft, -1, 1);
            backLeft = Range.clip(backLeft, -1, 1);
            backRight = Range.clip(backRight, -1, 1);
        } else {
            frontRight = 0;
            frontLeft = 0;
            backRight = 0;
            backLeft = 0;
        }
        telemetry.addData("FR", frontRight);
        telemetry.addData("FL", frontLeft);
        telemetry.addData("BR", backRight);
        telemetry.addData("BL", backLeft);
        double tolerance = .1;
        FR.setPower(frontRight*moveSpeed);
        FL.setPower(frontLeft*moveSpeed);
        BR.setPower(backRight*moveSpeed);
        BL.setPower(backLeft*moveSpeed);

        //glyph wheels
        if (gamepad2.right_bumper) succ(1);
        else if (gamepad2.left_bumper) succ(-1);
        else succ(0);

        //grabber 1 IS LEFT, 2 IS RIGHT
        if (gamepad2.dpad_down){ //in
            GlyphServo1.setPosition(.17); //make left tighter
            GlyphServo2.setPosition(.2);
        }
        //dpad press flat
        else if (gamepad2.dpad_up) { //out
            GlyphServo1.setPosition(.275);
            GlyphServo2.setPosition(.28);
        }
        else if (gamepad2.dpad_left){ //left in
            GlyphServo2.setPosition(.4);
            GlyphServo1.setPosition(.25);
        }
        else if (gamepad2.dpad_right) { //right in
            GlyphServo2.setPosition(.27);
            GlyphServo1.setPosition(.4);
        }
        else { //out
            GlyphServo1.setPosition(.375);
            GlyphServo2.setPosition(.375);
        }
        if (gamepad2.right_stick_button) GlyphWheel3.setPower(1);
        else GlyphWheel3.setPower(0);

        if (gamepad2.a) push();
        PushServo.setPosition(0);

        //lift
        if((gamepad2.right_stick_y > -.1) && !(gamepad2.right_stick_y<.1)) liftUp();
        else if (gamepad2.right_stick_y < .1 && !(gamepad2.right_stick_y>-.1))liftDown();
        if (gamepad1.right_stick_x<-.1) liftStop();
        else liftNoPower();

        telemetry.addData("Glyph Servo 1", GlyphServo1.getPosition());
        telemetry.addData("Glyph Servo 2", GlyphServo2.getPosition());
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
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