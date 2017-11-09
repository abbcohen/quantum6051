/**
 * Created by student on 10/19/17.
 */
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



@Autonomous(name="Concept: VuMark Id", group ="Concept")
@Disabled
public class auto1Kara extends LinearOpMode {
    public static final String TAG = "Vuforia Test1";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Aah86E7/////AAAAGbepZIhx50WRm54994wdq18t4COea3bQFWGU5u3W/YOQwaE9udSjupVYYTtS5sxoMLW2CSsutGAqpEwBazPqGHWH8hdOLLA7GzyP+IcgwJl+6aEtW9bu1AnqhzwYq8KgQuK17ZdyT/od9p106n38oQ11Z7fS9xLmxVvJzS2FzQPl4h/3X2ejvQTSPLTeNg+IbGFB6JPlG2WGcqaAYd+6f95Ly//mfSzevco9SvoYH1lann//GJGXbyM0G7NUd/SKCNPr/EpyNkG6+iq7ie6p1CATVExOqoQUSsYy2ygZGpNZResLn2xH5wAPUDKGuqSJmlfByC8Odwwlz8GsBi02uT5cry0XZf5tbeUOzpILzlGU";
    }

}

