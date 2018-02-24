package org.firstinspires.ftc.teamcode.autonomous.finalAutos;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

public class Robot {

    //COLOR SENSOR VARIABLES
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    ColorSensor colorSensor;
    DistanceSensor sensorDistance;


    //SYSTEM VARIABLES
    ElapsedTime clock = new ElapsedTime();
    RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    //MOTOR VARIABLES AND THINGS
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private DcMotor LiftMotor = null;
    public Servo JewelServo = null;
    private Servo GlyphServoL = null;
    private Servo GlyphServoR = null;
    private DcMotor GlyphWheelL = null;
    private DcMotor GlyphWheelR = null;
    private double moveSpeed = .25;
    private double turnSpeed = .25;

    //GYRO THINGS
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public void init(LinearOpMode o){
        //SYSTEM THINGS
        opMode = o;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        //HARDWARE THINGS
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        GlyphWheelL = hardwareMap.get(DcMotor.class, "GlyphWheelL");
        GlyphWheelR = hardwareMap.get(DcMotor.class, "GlyphWheelR");
        JewelServo.setDirection(Servo.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheelL.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheelR.setDirection(DcMotor.Direction.REVERSE);
        GlyphServoL = hardwareMap.get(Servo.class, "GlyphServoL");
        GlyphServoL.setDirection(Servo.Direction.REVERSE);
        GlyphServoR = hardwareMap.get(Servo.class, "GlyphServoR");
        GlyphServoR.setDirection(Servo.Direction.FORWARD);
        //Braking
        final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
        FR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        FL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BR.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        BL.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        LiftMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

        //GYRO THINGS
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //COLORSENSOR MAYBE VUFORIA THINGS
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // wait for the start button to be pressed.
    }
    public void idle(){
        opMode.idle();
    }
    public boolean opModeIsActive(){
        return opMode.opModeIsActive();
    }
    public double getRuntime(){
        return opMode.getRuntime();
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

    public void glyphWheels(double speed) {
        GlyphWheelL.setPower(speed);
        GlyphWheelR.setPower(speed);
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

    public void turnClockwise(double speed){
        FR.setPower(-speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(-speed);
    }

    public void turnCounterClockwise(double speed){
        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
    }
    public void turnCounterClockwise() {
        FR.setPower(turnSpeed);
        FL.setPower(turnSpeed);
        BR.setPower(turnSpeed);
        BL.setPower(turnSpeed);
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

    public void grabberIn(){
        GlyphServoL.setPosition(.2);
        GlyphServoR.setPosition(.25);
    }

    public void grabberOut(){
        GlyphServoL.setPosition(.375);
        GlyphServoR.setPosition(.375);
    }

    public void lift(int power){
        LiftMotor.setPower(-power);
    }

    public RelicRecoveryVuMark getPicto() { //function to figure out which column it is
        OpenGLMatrix lastLocation = null;

        VuforiaLocalizer vuforia;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "Aeba4Qn/////AAAAGahNOxzreUE8nItPWzsrOlF7uoyrR/qbjue3kUmhxZcfZMSd5MRyEY+3uEoVA+gpQGz5KyP3wEjBxSOAb4+FBYMZ+QblFU4byMG4+aiI+GeeBA+RatQXVzSduRBdniCW4qehTnwS204KTUMXg1ioPvUlbYQmqM5aPMx/2xnYN1b+htNBWV0Bc8Vkyspa0NNgz7PzF1gozlCIc9FgzbzNYoOMhqSG+jhKf47SZQxD6iEAtj5iAkWBvJwFDLr/EfDfPr3BIA6Cpb4xaDc0t4Iz5wJ/p4oLRiEJaLoE/noCcWFjLmPcw9ccwYXThtjC+7u0DsMX+r+1dMikBCZCWWkLzEyjWzy3pOOR3exNRYGZ0vzr";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //look through back camera
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        double vuStartTime = getRuntime();
        while (getRuntime() - vuStartTime < 3) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        return vuMark;
    }
    double currentAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    public void turnAngle(double angle){
        if(angle > 0) turnAngleCW(angle);
        if(angle < 0) turnAngleCCW(-angle);
    }
    public void turnAngleCW(double angle) {
        double startingAngle = currentAngle();
        while(getAngleDiff(startingAngle,currentAngle())<angle){
            double difference = ((angle-getAngleDiff(startingAngle,currentAngle()))/(angle*4));
            telemetry.addData("difference",difference);
            telemetry.update();
            if(difference > .15) turnClockwise(difference);
            else turnClockwise(.15);
        }
        driveStop();
    }
    public void turnAngleCCW(double angle) {
        double startingAngle = currentAngle();
        while(getAngleDiff(startingAngle,currentAngle())<angle){
            double difference = ((angle-getAngleDiff(startingAngle,currentAngle()))/(angle*4));
            telemetry.addData("difference",difference);
            telemetry.update();
            if(difference > .15) turnCounterClockwise(difference);
            else turnCounterClockwise(.15);
        }
        driveStop();
    }

    public double getAngleDiff(double angle1, double angle2) {
        if(Math.abs(angle1 - angle2) < 180.0)
            return Math.abs(angle1-angle2);
        else if(angle1 > angle2)
        {
            angle1 -= 360;
            return Math.abs(angle2-angle1);
        }
        else
        {
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
}
