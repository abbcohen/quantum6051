package org.firstinspires.ftc.teamcode.autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "wheeltest", group = "Sensor")
public class wheeltest extends LinearOpMode {
    ElapsedTime clock = new ElapsedTime();
    RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;

    ColorSensor colorSensor;
    DistanceSensor sensorDistance;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private Servo JewelServo = null;
    //    private Servo GlyphServoR = null;
//    private Servo GlyphServoL = null;
    private DcMotor GlyphWheel1 = null;
    private DcMotor GlyphWheel2 = null;

    private double moveSpeed = .25;
    private double turnSpeed = .25;

    @Override
    public void runOpMode() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        GlyphWheel1 = hardwareMap.get(DcMotor.class, "GlyphWheel1");
        GlyphWheel2 = hardwareMap.get(DcMotor.class, "GlyphWheel2");

        JewelServo.setDirection(Servo.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        GlyphWheel1.setDirection(DcMotor.Direction.FORWARD);
        GlyphWheel2.setDirection(DcMotor.Direction.REVERSE);

        moveTime(3,2);
        moveTime(4,2);
        moveTime(5,2);
        moveTime(6,2);


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
}