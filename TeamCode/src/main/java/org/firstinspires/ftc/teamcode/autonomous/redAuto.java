
package org.firstinspires.ftc.teamcode.autonomous;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import static java.lang.Thread.sleep;

@Autonomous(name="Red Juul Auto", group="Iterative Opmode")
public class redAuto extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WheelOne = null;
    private DcMotor WheelTwo = null;
    private DcMotor WheelThree = null;
    private DcMotor WheelZero = null;
    private DcMotor LiftMotor = null;
    private Servo GlyphServo = null;
    private Servo JewelServo = null;
    private double moveSpeed = .75;
    private double turnSpeed = .5;
    private double liftSpeed = 1;
    private boolean slomo;
    public double servoPos = .5;
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
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        WheelOne.setDirection(DcMotor.Direction.FORWARD);
        WheelTwo.setDirection(DcMotor.Direction.REVERSE);
        WheelThree.setDirection(DcMotor.Direction.REVERSE);
        WheelZero.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setDirection(DcMotor.Direction.REVERSE);
        //Servo initialization
        GlyphServo = hardwareMap.get(Servo.class, "GlyphServo");
        GlyphServo.setDirection(Servo.Direction.REVERSE);
        GlyphServo.scaleRange(0,2);
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        JewelServo.setDirection(Servo.Direction.REVERSE);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        boolean bLedOn = true;
        colorSensor.enableLed(bLedOn);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void moveForward() {
        WheelOne.setPower(.25);
        WheelTwo.setPower(.25);
        WheelThree.setPower(.25);
        WheelZero.setPower(.25);
    }
    public void moveBackward() {
        WheelOne.setPower(-.25);
        WheelTwo.setPower(-.25);
        WheelThree.setPower(-.25);
        WheelZero.setPower(-.25);
    }
    public void moveLeft() {
        WheelOne.setPower(.25);
        WheelTwo.setPower(.25);
        WheelThree.setPower(-.25);
        WheelZero.setPower(-.25);
    }
    public void moveRight() {
        WheelOne.setPower(-.25);
        WheelTwo.setPower(-.25);
        WheelThree.setPower(.25);
        WheelZero.setPower(.25);
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
    public void driveStop(){
        WheelOne.setPower(0);
        WheelTwo.setPower(0);
        WheelThree.setPower(0);
        WheelZero.setPower(0);
    }public void grabber(double pos) {
        if(pos==1){
            //OUT left
            GlyphServo.setPosition(.41);
        }
        else if (pos==0) {
            //IN right
            GlyphServo.setPosition(.54);
        }
    }
    public void jewel(char position){
        if(position == 'u') JewelServo.setPosition(90);
        if(position =='d') JewelServo.setPosition(0);
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
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
        // send the info back to driver station using telemetry function.

        int red=0;
        int blue=0;
        for (int x = 0; x < 100; x++) {
            if(colorSensor.red()>colorSensor.blue()){
                red++;
            }
            else {
                blue++;
            }
        }

        double initialtime =  getRuntime();

        if(red>blue){
            while(getRuntime() < (initialtime + 1.5)) {
                //turnClockwise();
            }
        } else{
            while(getRuntime() < (initialtime + 1.5)) {
                //turnCounterClockwise();
            }
        }

        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("r", red);
        telemetry.addData("b", blue);
        telemetry.addData("hsv", hsvValues);


//        Color.RGBToHSV(colorSensor.red() * 255/800, colorSensor.green() * 255/800, colorSensor.blue() * 255/800, hsvValues);
//            if (hsvValues[0] > 140 && hsvValues[0] < 310) {
//
//
//            }
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //stop everything
    }
}