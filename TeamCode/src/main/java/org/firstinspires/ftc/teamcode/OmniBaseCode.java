package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DOBBY GAME TIME", group="Iterative Opmode")
public class OmniBaseCode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //declare motors and servos
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private DcMotor LiftMotor = null;
    private DcMotor RelicMotor = null;
    private DcMotor GlyphWheel1 = null;
    private DcMotor GlyphWheel2 = null;
    private Servo GlyphServoL = null;
    private Servo GlyphServoR = null;
    private Servo JewelServo = null;
    private Servo PushServo = null;
    private Servo RelicWristServo = null;
    private Servo RelicFingerServo = null;

    //declare variables
    private double moveSpeed = .9;
    private double relicSpeed = .7;
    private double liftSpeed = 1;
    private boolean slomo = false;
    private boolean ultraslomo = false;
    private boolean readhand = false;
    private boolean readwrist = false;
    private boolean readframe = false;
    private boolean defaultFrame = true;


    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //initialize motors and servos
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        RelicMotor = hardwareMap.get(DcMotor.class, "RelicMotor");
        GlyphWheel1 = hardwareMap.get(DcMotor.class, "GlyphWheel1");
        GlyphWheel2 = hardwareMap.get(DcMotor.class, "GlyphWheel2");
        GlyphServoL = hardwareMap.get(Servo.class, "GlyphServoL");
        GlyphServoR = hardwareMap.get(Servo.class, "GlyphServoR");
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        PushServo = hardwareMap.get(Servo.class, "PushServo");
        RelicWristServo = hardwareMap.get(Servo.class, "RelicWristServo");
        RelicFingerServo = hardwareMap.get(Servo.class, "RelicFingerServo");

        //set motor and servo directions
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
        RelicWristServo.setDirection(Servo.Direction.FORWARD);
        RelicFingerServo.setDirection(Servo.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    public void stopDriving() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void glyphWheels(double speed) {
        GlyphWheel1.setPower(speed);
        GlyphWheel2.setPower(speed);
    }

    public void liftMove() {
        LiftMotor.setPower(gamepad2.right_stick_y * liftSpeed);
    }

    public void liftStop() {
        LiftMotor.setPower(.05);
    }

    public void relicArmMove() {
        RelicMotor.setPower(gamepad2.left_stick_y * relicSpeed);
    }

    public void relicArmStop() {
        RelicMotor.setPower(.05);
    }

    public void FlipRelicWrist() {
        if (RelicWristServo.getPosition()>.5) RelicWristServo.setPosition(.1);
        else RelicWristServo.setPosition(.9); //0 is down, 1 is up
    }

    public void RelicHandClose(){
        if (RelicFingerServo.getPosition()>.6) RelicFingerServo.setPosition(.5);
        else RelicFingerServo.setPosition(.7); //.1 is in, .8 is out
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        JewelServo.setPosition(.5);
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
        if (slomo) moveSpeed = .4;
        else moveSpeed = .9;
//
//        //ultra slowdo
//        if (gamepad1.right_trigger > 0.5)  ultraslomo = true;
//        else ultraslomo = false;
//        if(ultraslomo) moveSpeed = .2;
//        else moveSpeed = .9;
//        //flip reference frame
//        if (gamepad1.y) {
//            if (!readframe) {
//                readframe = true;
//                defaultFrame=!defaultFrame;
//            }
//        } else readframe = false;

        //driving (omni wheel magic)
        double gamepad1LeftY = -gamepad1.left_stick_y;
        double gamepad1LeftX = gamepad1.left_stick_x;
        double gamepad1RightX = gamepad1.right_stick_x * .7;
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
        //scale the omni magic speeds to the speeds we like
        if(defaultFrame) {
            FR.setPower(frontRight * moveSpeed);
            FL.setPower(frontLeft * moveSpeed);
            BR.setPower(backRight * moveSpeed);
            BL.setPower(backLeft * moveSpeed);
        }else{
            FR.setPower(backRight * moveSpeed);
            FL.setPower(frontRight * moveSpeed);
            BR.setPower(backLeft * moveSpeed);
            BL.setPower(frontLeft * moveSpeed);
        }

        //stop driving
        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0)
            stopDriving();

        //glyph intake wheels
        if (gamepad2.right_bumper) glyphWheels(1); //pull glyph in
        else if (gamepad2.left_bumper) glyphWheels(-1); //push glyph out
        else glyphWheels(0); //stop glyph wheels

        //grabber
        if (gamepad2.dpad_down) { //in
            GlyphServoL.setPosition(.2);
            GlyphServoR.setPosition(.23);
        } else if (gamepad2.dpad_up) { //out
            GlyphServoL.setPosition(.375);
            GlyphServoR.setPosition(.375);
        } else if (gamepad2.dpad_left) { //left in
            GlyphServoL.setPosition(.25);
            GlyphServoR.setPosition(.4);
        } else if (gamepad2.dpad_right) { //right in
            GlyphServoL.setPosition(.4);
            GlyphServoR.setPosition(.27);
        } else { //out
            GlyphServoL.setPosition(.275);
            GlyphServoR.setPosition(.28);
        }

        //glyph pusher plate
        if (gamepad2.right_trigger>.2) PushServo.setPosition(.3);//out (up)
        else PushServo.setPosition(.7); // in (down)

        //lift
        if ((gamepad2.right_stick_y > .1) || (gamepad2.right_stick_y < -.1)) liftMove();
        else if (gamepad2.right_stick_x < -.1) liftStop();
        else LiftMotor.setPower(0);

        //relic
        if ((gamepad2.left_stick_y > .1) || (gamepad2.left_stick_y < -.1)) relicArmMove();
        else relicArmStop();
        if (gamepad2.x) {
            if (!readhand) {
                readhand = true;
                RelicHandClose();
            }
        }else readhand=false;

        if (gamepad2.y) {
            if (!readwrist) {
                readwrist = true;
                FlipRelicWrist();
                }
        } else readwrist = false;

            //Keep the jewel servo up
            JewelServo.setPosition(.5);

            //console
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FR", frontRight);
            telemetry.addData("FL", frontLeft);
            telemetry.addData("BR", backRight);
            telemetry.addData("BL", backLeft);
            telemetry.addData("relicFinger", RelicFingerServo.getPosition());
            telemetry.addData("relicWrist", RelicWristServo.getPosition());
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