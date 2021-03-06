package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

@Autonomous(name = "double red 1", group = "Sensor")
@Disabled //SO YOU CANNOT RUN IT
public class doubleRed1 extends LinearOpMode {

    // The IMU sensor object
    BNO055IMU imu;

    ElapsedTime clock = new ElapsedTime();
    ColorSensor colorSensor;
    DistanceSensor sensorDistance;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private Servo JewelServo = null;
    //    private Servo GlyphServoL = null;
//    private Servo GlyphServoR = null;
    private DcMotor GlyphWheel1 = null;
    private DcMotor GlyphWheel2 = null;
    private double moveSpeed = .25;
    private double turnSpeed = .2;

    @Override
    public void runOpMode() {
        setupHardware();
        setupSoftware();
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


        // wait for the start button to be pressed.
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        JewelServo.setPosition(70);
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.update();

        //read color
        int red = 0;
        int blue = 0;
        for (int i = 0; i < 40; i++) {
            if (colorSensor.red() > colorSensor.blue()) red++;
            if (colorSensor.red() < colorSensor.blue()) blue++;
            telemetry.update();
        }

        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("RED", red);
        telemetry.addData("BLUE", blue);

        double initialAngle = angle();

        //knock off jewel
        if (red > blue) {
            telemetry.addData("Red Wins!", colorSensor.red());
            telemetry.update();
            turn(5, "counterclockwise");
        } else {
            telemetry.addData("Blue Wins!", colorSensor.red());
            telemetry.update();
            turn(5, "clockwise");
        }
        telemetry.addData("checkpoint", "knocked");
        telemetry.update();

        JewelServo.setPosition(0);
        moveTime(0,1);

        //turn back to initial position
        if (red > blue) {
            turn(5, "clockwise");
        } else if (blue > red) {
            turn(5, "counterclockwise");
        }
        telemetry.addData("checkpoint", "turn back");
        telemetry.update();
        moveTime(0,1);

        //Move off the stone
        moveTime(4,2);

        telemetry.addData("checkpoint", "at the box");
        telemetry.update();

        moveTime(0,1);

        //turn to face cryptobox
        turn(90, "clockwise");
        turn(90, "clockwise");

        //move forward
        moveTime(1, 1.2);

        //release glyph
        glyphWheels(-1);

        //move back
        moveTime(2, .25);

        //push back in
        moveTime(1, .3);

        //move back out
        moveTime(2, 1);

        //turn to the stack
        turn(90, "clockwise");
        turn(90, "clockwise");

        //turn on the intake
        glyphWheels(1);

        //drive to center
        moveTime(1, 4);

        //turn back to the box
        turn(90, "clockwise");
        turn(90, "clockwise");

        //drive to the wall
        moveTime(1, 4);

        //turn back to a new column
        turn(10, "clockwise");

        //drive to the wall`
        moveTime(1, .8);

        //release glyph
        moveTime(8, .28);

        //pause
        moveTime(0, 1);

        //move back
        moveTime(2, .25);

        //push back in
        moveTime(1, .3);

        //move back out
        moveTime(2, .3);
    }

    public void moveTime(int dir, double time) {
        double startTime = 0;
        if (dir == 0) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                driveStop();
            }
        }
        if (dir == 1) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                moveForward();
            }

        }
        if (dir == 2) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                moveBackward();
            }
        }
        if (dir == 3) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                moveLeft();
            }
        }
        if (dir == 4) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                moveRight();
            }
        }
        if (dir == 5) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                turnClockwise();
            }
        }
        if (dir == 6) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                turnCounterClockwise();
            }
        }
        if (dir == 7) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                glyphWheels(.6); //pull in
            }
        }
        if (dir == 8) {
            startTime = getRuntime();
            while (getRuntime() < startTime + time) {
                glyphWheels(-.6); //pull in
            }
        }
    }

    public void glyphWheels(double speed) { //1 in, -1 out
        GlyphWheel1.setPower(speed);
        GlyphWheel2.setPower(speed);
    }

    public void moveForward() {
        FR.setPower(moveSpeed);
        FL.setPower(-moveSpeed);
        BR.setPower(moveSpeed);
        BL.setPower(-moveSpeed);
    }

    public void moveBackward() {
        FR.setPower(-moveSpeed);
        FL.setPower(moveSpeed);
        BR.setPower(-moveSpeed);
        BL.setPower(moveSpeed);
    }

    public void moveLeft() {
        FR.setPower(moveSpeed);
        FL.setPower(moveSpeed);
        BR.setPower(-moveSpeed);
        BL.setPower(-moveSpeed);
    }

    public void moveRight() {
        FR.setPower(-moveSpeed);
        FL.setPower(-moveSpeed);
        BR.setPower(moveSpeed);
        BL.setPower(moveSpeed);
    }

    public void turnClockwise() {
        FR.setPower(-turnSpeed);
        FL.setPower(-turnSpeed);
        BR.setPower(-turnSpeed);
        BL.setPower(-turnSpeed);
    }

    public void turnCounterClockwise() {
        FR.setPower(turnSpeed);
        FL.setPower(turnSpeed);
        BR.setPower(turnSpeed);
        BL.setPower(turnSpeed);
    }
    public void turnSpeed(double speed) {
        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
    }
    public void moveDirection(int angle){
        double[] array = new double[4];
        int largest = 0;
        array[0] = Math.cos(Math.toRadians(angle+135)); //FL
        array[1] = Math.cos(Math.toRadians(angle+135+90)); //BL
        array[2] = Math.cos(Math.toRadians(angle+135+180)); //BR
        array[3] = Math.cos(Math.toRadians(angle+135+270)); //FR
        //Following lines scale
        for(int a = 1; a < 4; a++) if(array[a]>array[a-1]) largest = a;
        double scale = 1/array[largest];
        for(int a = 0; a < 4; a++) array[a] = array[a]*scale*moveSpeed;
        //Set Power
        FL.setPower(array[0]);
        BL.setPower(array[1]);
        BR.setPower(array[2]);
        FR.setPower(array[3]);
    }

    public void driveStop() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void delay(int time) {
        double delayStartTime = clock.milliseconds();
        while (clock.milliseconds() - delayStartTime < time) {
        }
    }
    double angle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    public void turn(double angle, String direction) {
        double startingAngle = angle();
        while (getAngleDiff(startingAngle, angle()) < angle) {
            telemetry.addData("not working", "plz");
            telemetry.addData("angleDiff", getAngleDiff(startingAngle, angle()));
            telemetry.addData("startingAngle", startingAngle);
            if (direction=="counterclockwise") {
                if (angle() - getAngleDiff(startingAngle, angle()) < 20.0) {
                    turnCounterClockwise();
                } else {
                    driveStop();
                }
            }else{
                if (angle() - getAngleDiff(startingAngle, angle()) < 20.0) {
                    turnClockwise();
                } else {
                    driveStop();
                }
            }
            telemetry.update();
        }
        driveStop();
    }

    public void otherturn(double angle){
        double startingAngle = angle();
        while(getAngleDiff(startingAngle,angle())<angle) {
            telemetry.addData("not working", "plz");
            telemetry.addData("angleDiff", getAngleDiff(startingAngle, angle()));
            telemetry.addData("startingAngle", startingAngle);
            if (angle>0) {
                if (angle() - getAngleDiff(startingAngle, angle()) < 20.0) {
                    turnCounterClockwise();
                } else {
                    driveStop();
                }
            }else{
                if (angle() - getAngleDiff(startingAngle, angle()) < 20.0) {
                    turnClockwise();
                } else {
                    driveStop();
                }
            }
            telemetry.update();
        }
    }

    public double getAngleDiff(double angle1, double angle2) {
        if(Math.abs(angle1 - angle2) < 180.0)
            return Math.abs(angle1-angle2);
        else if(angle1 > angle2) {
            angle1 -= 360;
            return Math.abs(angle2-angle1);
        } else {
            angle2 -= 360;
            return Math.abs(angle1-angle2);
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void correctPosition(double initialAngle) {
        while (getAngleDiff(initialAngle, angle()) > 0) {
            telemetry.addData("not working", "plz");
            telemetry.addData("angleDiff", getAngleDiff(initialAngle, angle()));
            telemetry.addData("startingAngle", initialAngle);
            if (angle() - getAngleDiff(initialAngle, angle()) < 20.0) {
                turnCounterClockwise();
            } else {
                driveStop();
            }
        }
    }
    public void setupSoftware(){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }
    public void setupHardware(){
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        GlyphWheel1 = hardwareMap.get(DcMotor.class, "GlyphWheel1");
        GlyphWheel2 = hardwareMap.get(DcMotor.class, "GlyphWheel2");

        JewelServo.setDirection(Servo.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheel1.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheel2.setDirection(DcMotor.Direction.REVERSE);

    }
}
