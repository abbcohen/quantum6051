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

@Autonomous(name = "final red 2", group = "Sensor")
public class red2 extends LinearOpMode {
    Robot dobby;
    public void runOpMode() {
        dobby = new Robot();
        dobby.init(this);
        telemetry.addLine("ready to go");
        telemetry.update();
        waitForStart();

        dobby.JewelServo.setPosition(70);
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (dobby.colorSensor.red() * dobby.SCALE_FACTOR),
                (int) (dobby.colorSensor.green() * dobby.SCALE_FACTOR),
                (int) (dobby.colorSensor.blue() * dobby.SCALE_FACTOR),
                dobby.hsvValues);

        dobby.column = dobby.getPicto();
        telemetry.addData("column", dobby.column);
        telemetry.update();

        //read color
        int red = 0;
        int blue = 0;
        for (int i = 0; i < 40; i++) {
            if (dobby.colorSensor.red() > dobby.colorSensor.blue()) red++;
            if (dobby.colorSensor.red() < dobby.colorSensor.blue()) blue++;
            telemetry.update();
        }

        telemetry.addData("Clear", dobby.colorSensor.alpha());
        telemetry.addData("Red  ", dobby.colorSensor.red());
        telemetry.addData("Green", dobby.colorSensor.green());
        telemetry.addData("Blue ", dobby.colorSensor.blue());
        telemetry.addData("Hue", dobby.hsvValues[0]);
        telemetry.addData("RED", red);
        telemetry.addData("BLUE", blue);

        //knock off jewel
        double jewelturntime = getRuntime();
        if (red > blue) {
            telemetry.addData("Red Wins!", dobby.colorSensor.red());
            telemetry.update();
            dobby.moveTime(5,.154);
        } else {
            telemetry.addData("Blue Wins!", dobby.colorSensor.red());
            telemetry.update();
            dobby.moveTime(6,.154);
        }

        dobby.JewelServo.setPosition(0);

        //turn back to initial position
        if(red>blue) {
            dobby.moveTime(6,.154);
        } else {
            dobby.moveTime(5,.154);
        }


        //MOVE TO THE CORRECT COLUMN
        if (dobby.column == RelicRecoveryVuMark.CENTER || dobby.column == RelicRecoveryVuMark.UNKNOWN) {
            dobby.moveTime(3, 1.6); //fill w center value
        } else if (dobby.column == RelicRecoveryVuMark.LEFT) {
            dobby.moveTime(3, 0); //fill w left value
        } else if (dobby.column == RelicRecoveryVuMark.RIGHT) {
            dobby.moveTime(3, 0);//fill w right value
        } else dobby.moveTime(3,1.763);


        //turn to face cryptobox
        dobby.moveTime(6,1.61);

        //move forward
        dobby.moveTime(1,1.2);

        //pause
        dobby.moveTime(0,1);

        //release glyph
        dobby.moveTime(8, .28);

        //pause
        dobby.moveTime(0,1);

        //move back
        dobby.moveTime(2,.25);

        //push back in
        dobby.moveTime(1,.3);

        //move back out
        dobby.moveTime(2,.25);
    }
}
