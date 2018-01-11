package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tele Op", group="Iterative Opmode")
public class OmniBaseCode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //declare motors and servos
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private DcMotor LiftMotor = null;
    private DcMotor GlyphWheel1 = null;
    private DcMotor GlyphWheel2 = null;
    private Servo GlyphServoL = null;
    private Servo GlyphServoR = null;
    private Servo JewelServo = null;
    private Servo PushServo = null;

    //declare variables
    private double moveSpeed = .75;
    private double turnSpeed = .5;
    private double liftSpeed = 1;
    private boolean slomo = false;
    private boolean push = false;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //init motors and servos
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        GlyphWheel1 = hardwareMap.get(DcMotor.class, "GlyphWheel1");
        GlyphWheel2 = hardwareMap.get(DcMotor.class, "GlyphWheel2");
        GlyphServoL = hardwareMap.get(Servo.class, "GlyphServoL");
        GlyphServoR = hardwareMap.get(Servo.class, "GlyphServoR");
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        PushServo = hardwareMap.get(Servo.class, "PushServo");

        //motor and servo directions
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        GlyphWheel1.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheel2.setDirection(DcMotor.Direction.REVERSE);
        GlyphServoL.setDirection(Servo.Direction.REVERSE);
        GlyphServoR.setDirection(Servo.Direction.FORWARD);
        JewelServo.setDirection(Servo.Direction.REVERSE);
        PushServo.setDirection(Servo.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    public void driveStop() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void succ(double speed) {
        GlyphWheel1.setPower(speed);
        GlyphWheel2.setPower(speed);
    }

    public void push(){
        if (push) PushServo.setPosition(.3);//out (up)
        else PushServo.setPosition(.7); // in (down)
        push=!push;
    }

    public void liftUp() {
        LiftMotor.setPower(gamepad2.right_stick_y * liftSpeed);
    }

    public void liftDown() {
        LiftMotor.setPower(gamepad2.right_stick_y * liftSpeed);
    }
    public void liftNoPower() {
        LiftMotor.setPower(0);
    }
    public void liftStop() {
        LiftMotor.setPower(.05);
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        JewelServo.setPosition(0);
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //slomo
        if (gamepad1.right_bumper) slomo = true;
        else slomo = false;
        if (slomo) moveSpeed = .6;
        else moveSpeed = .75;

        //driving (omni wheel magic)
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
        double tolerance = .1;
        FR.setPower(frontRight * moveSpeed);
        FL.setPower(frontLeft * moveSpeed);
        BR.setPower(backRight * moveSpeed);
        BL.setPower(backLeft * moveSpeed);

        //stop driving
        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0)
            driveStop();

        //glyph intake wheels
        if (gamepad2.right_bumper) succ(1);
        else if (gamepad2.left_bumper) succ(-1);
        else succ(0);

        //grabber
        if (gamepad2.dpad_down) { //in
            GlyphServoL.setPosition(.17);
            GlyphServoR.setPosition(.2);
        } else if (gamepad2.dpad_up) { //out
            GlyphServoL.setPosition(.275);
            GlyphServoR.setPosition(.28);
        } else if (gamepad2.dpad_left) { //left in
            GlyphServoL.setPosition(.25);
            GlyphServoR.setPosition(.4);
        } else if (gamepad2.dpad_right) { //right in
            GlyphServoL.setPosition(.4);
            GlyphServoR.setPosition(.27);
        } else { //out
            GlyphServoL.setPosition(.375);
            GlyphServoR.setPosition(.375);
        }

        //glyph pusher
        if (gamepad2.a) push();

        //lift
        if ((gamepad2.right_stick_y > -.1) && !(gamepad2.right_stick_y < .1)) liftUp();
        else if (gamepad2.right_stick_y < .1 && !(gamepad2.right_stick_y > -.1)) liftDown();
        if (gamepad1.right_stick_x < -.1) liftStop();
        else liftNoPower();

        //jewel servo up
        JewelServo.setPosition(0);

        //console
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("FR", frontRight);
        telemetry.addData("FL", frontLeft);
        telemetry.addData("BR", backRight);
        telemetry.addData("BL", backLeft);
        telemetry.addData("Glyph Servo 1", GlyphServoL.getPosition());
        telemetry.addData("Glyph Servo 2", GlyphServoR.getPosition());
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("right bumper", gamepad2.right_bumper);
        telemetry.addData("left bumper", gamepad2.left_bumper);
        telemetry.addData("lift", gamepad2.right_stick_y);
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}