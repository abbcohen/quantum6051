package org.firstinspires.ftc.teamcode.autonomous.finalAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "gyrotest", group = "Sensor")
public class gyroTesting extends LinearOpMode {
    Robot dobby;
    public void runOpMode() {
        dobby = new Robot();
        dobby.init(this);
        telemetry.addLine("ready to go");
        telemetry.update();
        waitForStart();
        dobby.turnAngle(90.0);
        dobby.turnAngle(-90.0);

    }
}