
package org.firstinspires.ftc.teamcode.autonomous;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name="Omni with servos", group="Iterative Opmode")
public class redAuto extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WheelOne = null;
    private DcMotor WheelTwo = null;
    private DcMotor WheelThree = null;
    private DcMotor WheelZero = null;
    private CRServo ServoFour = null;
    private CRServo ServoFive = null;
    private double moveSpeed = .75;
    private double turnSpeed = .5;
    private ColorSensor colorSensor;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        WheelOne  = hardwareMap.get(DcMotor.class, "WheelOne");
        WheelTwo  = hardwareMap.get(DcMotor.class, "WheelTwo");
        WheelThree = hardwareMap.get(DcMotor.class, "WheelThree");
        WheelZero = hardwareMap.get(DcMotor.class, "WheelZero");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        WheelOne.setDirection(DcMotor.Direction.FORWARD);
        WheelTwo.setDirection(DcMotor.Direction.REVERSE);
        WheelThree.setDirection(DcMotor.Direction.REVERSE);
        WheelZero.setDirection(DcMotor.Direction.FORWARD);
        //Servo initialization
        ServoFour = hardwareMap.get(CRServo.class, "ServoFour");
        ServoFive = hardwareMap.get(CRServo.class, "ServoFive");
        ServoFour.setDirection(CRServo.Direction.REVERSE);
        ServoFive.setDirection(CRServo.Direction.FORWARD);

        colorSensor = hardwareMap.colorSensor.get("color sensor");
        boolean bLedOn = true;
        colorSensor.enableLed(bLedOn);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void moveForward() {
        WheelOne.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelTwo.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelThree.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelZero.setPower(gamepad1.left_stick_y*moveSpeed);
    }
    public void moveBackward() {
        WheelOne.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelTwo.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelThree.setPower(gamepad1.left_stick_y*moveSpeed);
        WheelZero.setPower(gamepad1.left_stick_y*moveSpeed);
    }
    public void moveLeft() {
        WheelOne.setPower(-gamepad1.left_stick_x*moveSpeed);
        WheelTwo.setPower(-gamepad1.left_stick_x*moveSpeed);
        WheelThree.setPower(gamepad1.left_stick_x*moveSpeed);
        WheelZero.setPower(gamepad1.left_stick_x*moveSpeed);
    }
    public void moveRight() {
        WheelOne.setPower(-gamepad1.left_stick_x*moveSpeed);
        WheelTwo.setPower(-gamepad1.left_stick_x*moveSpeed);
        WheelThree.setPower(gamepad1.left_stick_x*moveSpeed);
        WheelZero.setPower(gamepad1.left_stick_x*moveSpeed);
    }
    public void turnClockwise() {
        WheelOne.setPower(gamepad1.right_stick_x*turnSpeed);
        WheelTwo.setPower(-gamepad1.right_stick_x*turnSpeed);
        WheelThree.setPower(-gamepad1.right_stick_x*turnSpeed);
        WheelZero.setPower(gamepad1.right_stick_x*turnSpeed);
    }
    public void turnCounterClockwise() {
        WheelOne.setPower(gamepad1.right_stick_x*turnSpeed);
        WheelTwo.setPower(-gamepad1.right_stick_x*turnSpeed);
        WheelThree.setPower(-gamepad1.right_stick_x*turnSpeed);
        WheelZero.setPower(gamepad1.right_stick_x*turnSpeed);
    }
    public void driveStop(){
        WheelOne.setPower(0);
        WheelTwo.setPower(0);
        WheelThree.setPower(0);
        WheelZero.setPower(0);
    }
    public void grabberIn(){
        ServoFour.setPower(-gamepad1.right_trigger);
        ServoFive.setPower(-gamepad1.right_trigger);
    }
    public void grabberOut(){
        ServoFour.setPower(gamepad1.left_trigger);
        ServoFive.setPower(gamepad1.left_trigger);
    }
    public void grabberStop(){
        ServoFour.setPower(0);
        ServoFive.setPower(0);
    }


    @Override
    public void init_loop() {
        WheelOne.setPower(0);
        WheelTwo.setPower(0);
        WheelThree.setPower(0);
        WheelZero.setPower(0);

    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {// initialize here
        runtime.reset();
        init();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
       //do stuff here
        // convert the RGB values to HSV values.
        int red=0;
        int blue=0;
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);

        Color.RGBToHSV(colorSensor.red() * 255/800, colorSensor.green() * 255/800, colorSensor.blue() * 255/800, hsvValues);
            if (hsvValues[0] > 140 && hsvValues[0] < 310) {


            }
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //stop everything
    }
}