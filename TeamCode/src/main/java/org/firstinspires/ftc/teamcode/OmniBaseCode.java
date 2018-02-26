package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp(name="DOBBY GAME TIME", group="Iterative Opmode")
public class OmniBaseCode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //declare variables
    private double moveSpeed = 1;
    private double liftSpeed = 1;
    private double relicSpeed = .75;
    private boolean slomo = false;
    private boolean relicHandPos = true;
    private boolean yJustPressed = false;
    private boolean relicWristPos = true;
    private boolean xJustPressed = false;
    private boolean downJustPressed = false;
    private boolean upJustPressed = false;
    private boolean rightJustPressed = false;
    private boolean leftJustPressed = false;

    //declare motors and servos
    private DcMotor FR, FL, BR, BL, LiftMotor, RelicMotor, GlyphWheelL, GlyphWheelR = null;
    private Servo GlyphServoL, GlyphServoR, PushServo, JewelServo, RelicWristServo, RelicHandServo = null;

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
        GlyphWheelL = hardwareMap.get(DcMotor.class, "GlyphWheelL");
        GlyphWheelR = hardwareMap.get(DcMotor.class, "GlyphWheelR");
        GlyphServoL = hardwareMap.get(Servo.class, "GlyphServoL");
        GlyphServoR = hardwareMap.get(Servo.class, "GlyphServoR");
        PushServo = hardwareMap.get(Servo.class, "PushServo");
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        RelicWristServo = hardwareMap.get(Servo.class, "RelicWristServo");
        RelicHandServo = hardwareMap.get(Servo.class, "RelicHandServo");

        //set motor and servo directions
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
        RelicMotor.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheelL.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheelR.setDirection(DcMotor.Direction.REVERSE);
        GlyphServoL.setDirection(Servo.Direction.REVERSE);
        GlyphServoR.setDirection(Servo.Direction.FORWARD);
        JewelServo.setDirection(Servo.Direction.REVERSE);
        PushServo.setDirection(Servo.Direction.REVERSE);
        RelicWristServo.setDirection(Servo.Direction.FORWARD);
        RelicHandServo.setDirection(Servo.Direction.FORWARD);

        //set motor braking on
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RelicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GlyphWheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GlyphWheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialization complete
        telemetry.addData("Status", "Initialized");
    }

    public void stopDriving() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void glyphWheels(double speed) {
        GlyphWheelL.setPower(speed);
        GlyphWheelR.setPower(speed);
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        stopDriving();
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

        //stop driving
        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) stopDriving();

        //driving slomo
        if (gamepad1.right_bumper) slomo = true;
        else slomo = false;
        if (slomo) moveSpeed = .4;
        else moveSpeed = .99;

        //set omniwheel speeds
        FR.setPower(frontRight * moveSpeed);
        FL.setPower(frontLeft * moveSpeed);
        BR.setPower(backRight * moveSpeed);
        BL.setPower(backLeft * moveSpeed);

        //glyph intake wheel
        if (gamepad2.right_bumper) glyphWheels(1); //pull in
        else if (gamepad2.left_bumper) glyphWheels(-1); //push out
        else glyphWheels(0); //stop

        //glyph grabbers
        if (gamepad2.dpad_down ) { //in
            GlyphServoL.setPosition(.2);
            GlyphServoR.setPosition(.25);
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
            GlyphServoR.setPosition(.3);
        }

        if (!gamepad2.dpad_down) downJustPressed = false;
        if (!gamepad2.dpad_left) leftJustPressed = false;
        if (!gamepad2.dpad_right) rightJustPressed = false;
        if (!gamepad2.dpad_up)  upJustPressed = false;

        //glyph pusher
        if (gamepad2.right_trigger>.2) PushServo.setPosition(.45); //out
        else PushServo.setPosition(.85); //in

        //lift motor
        if ((gamepad2.right_stick_y > .1) || (gamepad2.right_stick_y < -.1)) LiftMotor.setPower(gamepad2.right_stick_y * liftSpeed);
        else LiftMotor.setPower(-.15);

        //relic motor
        if ((gamepad2.left_stick_y > .1) || (gamepad2.left_stick_y < -.1)) RelicMotor.setPower(gamepad2.left_stick_y * relicSpeed);
        else RelicMotor.setPower(0);

        //relic wrist
        if (gamepad2.y && !yJustPressed) {
            if (relicWristPos) RelicWristServo.setPosition(.9);
            else RelicWristServo.setPosition(.1);
            relicWristPos = !relicWristPos;
            yJustPressed = true;
        }

        if (!gamepad2.y) yJustPressed = false;

        //relic hand
        if (gamepad2.x && !xJustPressed) {
            if (relicHandPos) RelicHandServo.setPosition(.62);
            else RelicHandServo.setPosition(.5);
            relicHandPos = !relicHandPos;
            xJustPressed = true;
        }

        if (!gamepad2.x) xJustPressed = false;

        //keep jewel servo up
        JewelServo.setPosition(.5);

        //print motor powers and servo positions
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("FR", FR.getPower());
        telemetry.addData("FL", FL.getPower());
        telemetry.addData("BR", BR.getPower());
        telemetry.addData("BL", BL.getPower());
        telemetry.addData("LiftMotor", LiftMotor.getPower());
        telemetry.addData("RelicMotor", RelicMotor.getPower());
        telemetry.addData("GlyphWheelL", GlyphWheelL.getPower());
        telemetry.addData("GlyphWheelR", GlyphWheelR.getPower());
        telemetry.addData("GlyphServoL", GlyphServoL.getPosition());
        telemetry.addData("GlyphServoR", GlyphServoR.getPosition());
        telemetry.addData("JewelServo", JewelServo.getPosition());
        telemetry.addData("RelicWristServo", RelicWristServo.getPosition());
        telemetry.addData("RelicHandServo", RelicHandServo.getPosition());
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {}
}