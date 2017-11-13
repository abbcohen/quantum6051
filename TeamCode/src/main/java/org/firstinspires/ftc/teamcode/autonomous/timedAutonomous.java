
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

@Autonomous(name = "timed blue auto", group = "Sensor")
public class timedAutonomous extends LinearOpMode {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     * <p>
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     * <p>
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     * <p>
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     */

    ColorSensor colorSensor;
    DistanceSensor sensorDistance;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WheelOne = null;
    private DcMotor WheelTwo = null;
    private DcMotor WheelThree = null;
    private DcMotor WheelZero = null;
    private Servo JewelServo = null;
    private Servo GlyphServo = null;

    @Override
    public void runOpMode() {
        WheelOne = hardwareMap.get(DcMotor.class, "WheelOne");
        WheelTwo = hardwareMap.get(DcMotor.class, "WheelTwo");
        WheelThree = hardwareMap.get(DcMotor.class, "WheelThree");
        WheelZero = hardwareMap.get(DcMotor.class, "WheelZero");
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        JewelServo.setDirection(Servo.Direction.REVERSE);
        WheelOne.setDirection(DcMotor.Direction.FORWARD);
        WheelTwo.setDirection(DcMotor.Direction.REVERSE);
        WheelThree.setDirection(DcMotor.Direction.REVERSE);
        WheelZero.setDirection(DcMotor.Direction.FORWARD);
        //Servo initialization
        GlyphServo = hardwareMap.get(Servo.class, "GlyphServo");
        GlyphServo.setDirection(Servo.Direction.REVERSE);
        GlyphServo.scaleRange(0,2);


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


        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        telemetry.update();


        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });


        //GRAB GLYPH
        GlyphServo.setPosition(.54);

        //JEWEL

        int red = 0;
        int blue = 0;
        int count = 0;
        for (int i = 0; i < 10; i++) {
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
            while (getRuntime() < (jewelturntime + .15)) {
                turnClockwise();
            }
        } else {
            telemetry.addData("Blue Wins!", colorSensor.red());
            telemetry.update();
            while (getRuntime() < (jewelturntime + .15)) {
                turnCounterClockwise();
            }
        }

        JewelServo.setPosition(0);

        //turn back to initial position
        double turnBacktime = getRuntime();
        if(red>blue) {
            while (getRuntime() < (turnBacktime + .15)) {
                turnCounterClockwise();
            }
        } else if(blue>red) {
            while (getRuntime() < (turnBacktime + .15)) {
                turnClockwise();
            }
        }

        //MOVE TO SAFE ZONE
        double safeZoneTime= getRuntime();
        while (getRuntime() < (safeZoneTime + 1)) {
            moveLeft();
        }


    }

    public void moveLeft() {
        WheelOne.setPower(.25);
        WheelTwo.setPower(.25);
        WheelThree.setPower(-.25);
        WheelZero.setPower(-.25);
    }
    public void turnClockwise() {
        WheelOne.setPower(.25);
        WheelTwo.setPower(-.25);
        WheelThree.setPower(-.25);
        WheelZero.setPower(.25);
    }

    public void turnCounterClockwise() {
        WheelOne.setPower(-.25);
        WheelTwo.setPower(.25);
        WheelThree.setPower(.25);
        WheelZero.setPower(-.25);
    }
}