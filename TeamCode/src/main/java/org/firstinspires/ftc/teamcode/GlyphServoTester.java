
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name="glyph only", group="Iterative Opmode")
public class GlyphServoTester extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo GlyphServo = null;
    public double servoPos = .5;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //Servo initialization
        GlyphServo = hardwareMap.get(Servo.class, "GlyphServo");
        //GlyphServo.setDirection(Servo.Direction.REVERSE);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        GlyphServo.scaleRange(0,2);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void grabber(double pos) {
        if(pos==0){
            GlyphServo.setPosition(.45);
        }
        else if (pos==1) {
            GlyphServo.setPosition(.65);
        }
    }


    @Override
    public void init_loop() {

    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        //GlyphServo.setPosition(GlyphServo.getPosition());

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //grabber
        if (gamepad2.right_bumper) grabber(0);
        else if (gamepad2.left_bumper)grabber(1);
        //GlyphServo.setPosition(GlyphServo.getPosition());

        telemetry.addData("Servo Position", GlyphServo.getPosition());
        telemetry.addData("right bumper", gamepad2.right_bumper);
        telemetry.addData("left bumper", gamepad2.left_bumper);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}