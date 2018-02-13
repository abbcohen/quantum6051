/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "Gyro test", group = "Sensor")
public class GyroTest extends LinearOpMode {ElapsedTime clock = new ElapsedTime();
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

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

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        turn(90);



    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    double angle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    public void turn(double angle) {
        telemetry.addData("test", "test");
        double startingAngle = angle();
        while (getAngleDiff(startingAngle, angle()) < angle) {
            telemetry.addData("not working", "plz");
            telemetry.addData("angleDiff", getAngleDiff(startingAngle, angle()));
            telemetry.addData("startingAngle", startingAngle);
            if (angle > 0) {
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

}
