package org.firstinspires.ftc.teamcode.autonomous.finalAutos;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "final red 1", group = "Sensor")
public class red1 extends LinearOpMode {
    Robot dobby;
    public void runOpMode() {
        dobby = new Robot();
        dobby.init(this);
        telemetry.addLine("ready to go");
        telemetry.update();
        waitForStart();

        //Get starting angle
        double veryStartAngle = dobby.currentAngle();

        //Flip Jewel Out
        dobby.JewelServo.setPosition(70);

        //Setup Color
        Color.RGBToHSV((int) (dobby.colorSensor.red() * dobby.SCALE_FACTOR),
                (int) (dobby.colorSensor.green() * dobby.SCALE_FACTOR),
                (int) (dobby.colorSensor.blue() * dobby.SCALE_FACTOR),
                dobby.hsvValues);

        //Read color
        int red = 0;
        int blue = 0;
        for (int i = 0; i < 40; i++) {
            if (dobby.colorSensor.red() > dobby.colorSensor.blue()) red++;
            if (dobby.colorSensor.red() < dobby.colorSensor.blue()) blue++;
            telemetry.update();
        }

        //Print color
        telemetry.addData("Clear", dobby.colorSensor.alpha());
        telemetry.addData("Red  ", dobby.colorSensor.red());
        telemetry.addData("Green", dobby.colorSensor.green());
        telemetry.addData("Blue ", dobby.colorSensor.blue());
        telemetry.addData("Hue", dobby.hsvValues[0]);
        telemetry.addData("RED", red);
        telemetry.addData("BLUE", blue);

        //Read Column
        dobby.column = dobby.getPicto();
        telemetry.addData("column", dobby.column);
        telemetry.addData("things were done",0);
        telemetry.update();

        //grab glyph
        dobby.grabberIn();
        dobby.lift(1);
        dobby.moveTime(0,.35);
        dobby.lift(0);

        //knock off jewel
        double jewelturntime = getRuntime();
//        if (red > blue) {
//            telemetry.addData("Red Wins!", dobby.colorSensor.red());
//            telemetry.update();
//            dobby.turnAngle(2);
//        } else {
//            telemetry.addData("Blue Wins!", dobby.colorSensor.red());
//            telemetry.update();
//            dobby.turnAngle(-2);
//        }

        //Flip Jewel In
        dobby.JewelServo.setPosition(0);
        dobby.moveTime(0,.2);

        //turn back to initial position
//        if(red>blue) {
//            dobby.turnAngle(-2);
//        } else {
//            dobby.turnAngle(2);
//        }
        dobby.moveTime(0,.2);

        //Drive off stone to center
        dobby.moveTime(4,2.2);
        dobby.moveTime(0,1);

        //Turn to original heading
        dobby.turnAngle(dobby.currentAngle()-veryStartAngle);
        dobby.moveTime(0,3);
        dobby.turnAngle(90);
        dobby.moveTime(0,1);
        dobby.turnAngle(90);
        dobby.moveTime(0,1);


        //Turn towards correct column
        if (dobby.column == RelicRecoveryVuMark.LEFT) {
            dobby.turnAngle(-10);
        } else if (dobby.column == RelicRecoveryVuMark.RIGHT) {
            dobby.turnAngle(10);
        }
        dobby.moveTime(0,1);

        //move forward
        dobby.moveTime(1,2.2);
        dobby.moveTime(0,1);

        //release glyph forever
        dobby.grabberOut();
        dobby.glyphWheels(-1);
        dobby.moveTime(0,1);

        //move back
        dobby.moveTime(2,.25);
        dobby.moveTime(0,.5);

        //push back in
        dobby.moveTime(1,.3);
        dobby.moveTime(0,.5);

        //move back out
        dobby.moveTime(2,.25);
        dobby.moveTime(0,.5);
    }
}
