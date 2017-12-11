package org.firstinspires.ftc.teamcode.autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "blue 2", group = "Sensor")
public class blueAuto2 extends LinearOpMode {


    ColorSensor colorSensor;
    DistanceSensor sensorDistance;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WheelOne = null;
    private DcMotor WheelTwo = null;
    private DcMotor WheelThree = null;
    private DcMotor WheelZero = null;
    private Servo JewelServo = null;
    private Servo GlyphServo1 = null;
    private Servo GlyphServo2 = null;
    private DcMotor GlyphWheel1 = null;
    private DcMotor GlyphWheel2 = null;
    private DcMotor LiftMotor = null;

    private double moveSpeed = .25;
    private double turnSpeed = .25;

    @Override
    public void runOpMode() {
        WheelOne = hardwareMap.get(DcMotor.class, "WheelOne");
        WheelTwo = hardwareMap.get(DcMotor.class, "WheelTwo");
        WheelThree = hardwareMap.get(DcMotor.class, "WheelThree");
        WheelZero = hardwareMap.get(DcMotor.class, "WheelZero");
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        GlyphWheel1 = hardwareMap.get(DcMotor.class, "GlyphWheel1");
        GlyphWheel2 = hardwareMap.get(DcMotor.class, "GlyphWheel2");

        JewelServo.setDirection(Servo.Direction.REVERSE);
        WheelOne.setDirection(DcMotor.Direction.FORWARD);
        WheelTwo.setDirection(DcMotor.Direction.REVERSE);
        WheelThree.setDirection(DcMotor.Direction.REVERSE);
        WheelZero.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheel1.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheel2.setDirection(DcMotor.Direction.REVERSE);
        //Servo initialization
        GlyphServo1 = hardwareMap.get(Servo.class, "GlyphServo1");
        GlyphServo1.setDirection(Servo.Direction.REVERSE);
        GlyphServo2 = hardwareMap.get(Servo.class, "GlyphServo2");
        GlyphServo2.setDirection(Servo.Direction.FORWARD);


        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // wait for the start button to be pressed.
        waitForStart();

        JewelServo.setPosition(70);
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);


        //read color
        int red = 0;
        int blue = 0;
        int count = 0;
        for (int i = 0; i < 25; i++) {
            if (colorSensor.red() > colorSensor.blue()) {
                red++;
            }
            if (colorSensor.red() < colorSensor.blue()) {
                blue++;
            }
            telemetry.update();
        }

        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("RED", red);
        telemetry.addData("BLUE", blue);

        //knock off jewel
        double jewelturntime = getRuntime();
        if (red > blue) {
            telemetry.addData("Red Wins!", colorSensor.red());
            telemetry.update();
            moveTime(5,.154);
        } else {
            telemetry.addData("Blue Wins!", colorSensor.red());
            telemetry.update();
            moveTime(6,.154);
        }

        JewelServo.setPosition(0);

        //turn back to initial position
        if(red>blue) {
            moveTime(6,.154);
        } else if(blue>red) {
            moveTime(5,.154);
        }

        //MOVE TO SAFE ZONE
        moveTime(3,1.761); //side
        moveTime(1, .2); //back

        //turn to face cryptobox
        moveTime(6,.9075);

        //move forward
        moveTime(1,1.2);

        //pause
        moveTime(0,1);

        //release glyph
        moveTime(8, .28);

        //pause
        moveTime(0,1);

        //move back
        moveTime(2,.25);

        //push back in
        moveTime(1,.3);

        //move back out
        moveTime(2,.25);


    }
    public void moveTime(int dir, double time){
        double startTime = 0;
        if(dir == 0){
            startTime = getRuntime();
            while(getRuntime() < startTime + time){
                driveStop();
            }
        }
        if(dir == 1){
            startTime = getRuntime();
            while(getRuntime() < startTime + time){
                moveForward();
            }

        }
        if(dir == 2){
            startTime = getRuntime();
            while(getRuntime() < startTime + time){
                moveBackward();
            }
        }
        if(dir == 3){
            startTime = getRuntime();
            while(getRuntime() < startTime + time){
                moveLeft();
            }
        }
        if(dir == 4){
            startTime = getRuntime();
            while(getRuntime() < startTime + time){
                moveRight();
            }
        }
        if(dir == 5){
            startTime = getRuntime();
            while(getRuntime() < startTime + time){
                turnClockwise();
            }
        }
        if(dir == 6){
            startTime = getRuntime();
            while(getRuntime() < startTime + time){
                turnCounterClockwise();
            }
        }
        if(dir == 7){
            startTime = getRuntime();
            while(getRuntime() < startTime + time){
                succ();
            }
        }
        if(dir == 8){
            startTime = getRuntime();
            while(getRuntime() < startTime + time){
                blow();
            }
        }
    }

    public void succ() {
        GlyphWheel1.setPower(.6);
        GlyphWheel2.setPower(.6);
    }
    public void nosucc() {
        GlyphWheel1.setPower(0);
        GlyphWheel2.setPower(0);
    }
    public void blow() {
        GlyphWheel1.setPower(-.6);
        GlyphWheel2.setPower(-.6);
    }
    public void grabber(double pos) {
        GlyphServo1.setPosition(pos);
        GlyphServo2.setPosition(pos);
    }
    public void moveForward() {
        WheelOne.setPower(-moveSpeed);
        WheelTwo.setPower(-moveSpeed);
        WheelThree.setPower(-moveSpeed);
        WheelZero.setPower(-moveSpeed);
    }
    public void moveBackward() {
        WheelOne.setPower(moveSpeed);
        WheelTwo.setPower(moveSpeed);
        WheelThree.setPower(moveSpeed);
        WheelZero.setPower(moveSpeed);
    }
    public void moveLeft() {
        WheelOne.setPower(moveSpeed);
        WheelTwo.setPower(moveSpeed);
        WheelThree.setPower(-moveSpeed);
        WheelZero.setPower(-moveSpeed);
    }
    public void moveRight() {
        WheelOne.setPower(-moveSpeed);
        WheelTwo.setPower(-moveSpeed);
        WheelThree.setPower(moveSpeed);
        WheelZero.setPower(moveSpeed);
    }
    public void turnClockwise() {
        WheelOne.setPower(turnSpeed);
        WheelTwo.setPower(-turnSpeed);
        WheelThree.setPower(-turnSpeed);
        WheelZero.setPower(turnSpeed);
    }
    public void turnCounterClockwise() {
        WheelOne.setPower(-turnSpeed);
        WheelTwo.setPower(turnSpeed);
        WheelThree.setPower(turnSpeed);
        WheelZero.setPower(-turnSpeed);
    }
    public void driveStop() {
        WheelOne.setPower(0);
        WheelTwo.setPower(0);
        WheelThree.setPower(0);
        WheelZero.setPower(0);
    }

    public void liftUp(double time) {
        double startTime = getRuntime();
        while (getRuntime() < startTime + time) {
            LiftMotor.setPower(1);
        }
    }

    public void liftDown(double time) {
        double startTime = getRuntime();
        while (getRuntime() < startTime + time) {
            LiftMotor.setPower(-1);
        }
    }

    public void liftStop(double time) {
        double startTime = getRuntime();
        while (getRuntime() < startTime + time) {
            LiftMotor.setPower(0);
        }
    }
}