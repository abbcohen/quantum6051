//
//package org.firstinspires.ftc.teamcode.autonomous;
//import android.app.Activity;
//import android.graphics.Color;
//import android.view.View;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//import java.util.Locale;
////import static java.lang.Thread.sleep;
//
//@Autonomous(name="theo is lame", group="Iterative Opmode")
//public class redAuto extends OpMode {
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor WheelOne = null;
//    private DcMotor WheelTwo = null;
//    private DcMotor WheelThree = null;
//    private DcMotor WheelZero = null;
//    private DcMotor LiftMotor = null;
//    private Servo GlyphServo = null;
//    private Servo JewelServo = null;
//    private double moveSpeed = .75;
//    private double turnSpeed = .5;
//    private double liftSpeed = 1;
//    private boolean slomo;
//    public double servoPos = .5;
//    private ColorSensor colorSensor;
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//        telemetry.addData("Status", "Initialized");
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        WheelOne  = hardwareMap.get(DcMotor.class, "WheelOne");
//        WheelTwo  = hardwareMap.get(DcMotor.class, "WheelTwo");
//        WheelThree = hardwareMap.get(DcMotor.class, "WheelThree");
//        WheelZero = hardwareMap.get(DcMotor.class, "WheelZero");
//        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        WheelOne.setDirection(DcMotor.Direction.FORWARD);
//        WheelTwo.setDirection(DcMotor.Direction.REVERSE);
//        WheelThree.setDirection(DcMotor.Direction.REVERSE);
//        WheelZero.setDirection(DcMotor.Direction.FORWARD);
//        LiftMotor.setDirection(DcMotor.Direction.REVERSE);
//        //Servo initialization
//        GlyphServo = hardwareMap.get(Servo.class, "GlyphServo");
//        GlyphServo.setDirection(Servo.Direction.REVERSE);
//        GlyphServo.scaleRange(0,2);
//        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
//        JewelServo.setDirection(Servo.Direction.REVERSE);
//        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
//
//        colorSensor = hardwareMap.colorSensor.get("colorSensor");
//        boolean bLedOn = true;
//        colorSensor.enableLed(bLedOn);
//        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
//
//    }
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    public void moveForward() {
//        WheelOne.setPower(.25);
//        WheelTwo.setPower(.25);
//        WheelThree.setPower(.25);
//        WheelZero.setPower(.25);
//    }
//    public void moveBackward() {
//        WheelOne.setPower(-.25);
//        WheelTwo.setPower(-.25);
//        WheelThree.setPower(-.25);
//        WheelZero.setPower(-.25);
//    }
//    public void moveLeft() {
//        WheelOne.setPower(.25);
//        WheelTwo.setPower(.25);
//        WheelThree.setPower(-.25);
//        WheelZero.setPower(-.25);
//    }
//    public void moveRight() {
//        WheelOne.setPower(-.25);
//        WheelTwo.setPower(-.25);
//        WheelThree.setPower(.25);
//        WheelZero.setPower(.25);
//    }
//    public void turnClockwise() {
//        WheelOne.setPower(.25);
//        WheelTwo.setPower(-.25);
//        WheelThree.setPower(-.25);
//        WheelZero.setPower(.25);
//    }
//    public void turnCounterClockwise() {
//        WheelOne.setPower(-.25);
//        WheelTwo.setPower(.25);
//        WheelThree.setPower(.25);
//        WheelZero.setPower(-.25);
//    }
//    public void driveStop(){
//        WheelOne.setPower(0);
//        WheelTwo.setPower(0);
//        WheelThree.setPower(0);
//        WheelZero.setPower(0);
//    }public void grabber(double pos) {
//        if(pos==1){
//            //OUT left
//            GlyphServo.setPosition(.41);
//        }
//        else if (pos==0) {
//            //IN right
//            GlyphServo.setPosition(.54);
//        }
//    }
//    public void jewel(char position){
//        if(position == 'u') JewelServo.setPosition(90);
//        if(position =='d') JewelServo.setPosition(0);
//    }
//
//
//    @Override
//    public void init_loop() {
//        WheelOne.setPower(0);
//        WheelTwo.setPower(0);
//        WheelThree.setPower(0);
//        WheelZero.setPower(0);
//
//    }
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//        init();
//       //do stuff here
//        // convert the RGB values to HSV values.
//// get a reference to the color sensor.
//        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
//
//        // get a reference to the distance sensor that shares the same name.
//        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");
//
//        // hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues[] = {0F, 0F, 0F};
//
//        // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;
//
//        // sometimes it helps to multiply the raw RGB values with a scale factor
//        // to amplify/attentuate the measured values.
//        final double SCALE_FACTOR = 255;
//
//        // get a reference to the RelativeLayout so we can change the background
//        // color of the Robot Controller app to match the hue detected by the RGB sensor.
//        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
//        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
//
//        // wait for the start button to be pressed.
//        waitForStart();
//
//        // loop and read the RGB and distance data.
//        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
//        while (true) {
//            // convert the RGB values to HSV values.
//            // multiply by the SCALE_FACTOR.
//            // then cast it back to int (SCALE_FACTOR is a double)
//            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
//                    (int) (colorSensor.green() * SCALE_FACTOR),
//                    (int) (colorSensor.blue() * SCALE_FACTOR),
//                    hsvValues);
//
//            // send the info back to driver station using telemetry function.
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Alpha", colorSensor.alpha());
//            telemetry.addData("Red  ", colorSensor.red());
//            telemetry.addData("Green", colorSensor.green());
//            telemetry.addData("Blue ", colorSensor.blue());
//            telemetry.addData("Hue", hsvValues[0]);
//
//            // change the background color to match the color detected by the RGB sensor.
//            // pass a reference to the hue, saturation, and value array as an argument
//            // to the HSVToColor method.
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
//                }
//            });
//
//            telemetry.update();
//        }
//
//        // Set the panel back to the default color
//        relativeLayout.post(new Runnable() {
//            public void run() {
//                relativeLayout.setBackgroundColor(Color.WHITE);
//            }
//        });
//
//        int red=0;
//        int blue=0;
//        int count=0;
//        for (int i = 0; i < 100; i++) {
//            if(colorSensor.red()>colorSensor.blue()){
//                red++;
//            }
//            if(colorSensor.red()<colorSensor.blue()){
//                blue++;
//            }
//        }
//
//
//        double initialtime =  getRuntime();
//        if(red>blue) {
//            while (getRuntime() < (initialtime + 1.5)) {
//                //turnClockwise();
//                telemetry.addData("Red Wins!", colorSensor.red());
//            }
//        } else{
//            while(getRuntime() < (initialtime + 1.5)) {
//                //turnCounterClockwise();
//                telemetry.addData("Blue Wins!", colorSensor.red());
//            }
//        }
//        telemetry.addData("Clear", colorSensor.alpha());
//        telemetry.addData("Red  ", colorSensor.red());
//        telemetry.addData("Green", colorSensor.green());
//        telemetry.addData("Blue ", colorSensor.blue());
//        telemetry.addData("Hue", hsvValues[0]);
//        telemetry.addData("color", red);
//        telemetry.addData("BLUE", blue); }
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//        //stop everything
//    }
//}