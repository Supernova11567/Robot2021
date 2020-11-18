package org.firstinspires.ftc.teamcode.Autonomous;

import com.vuforia.Vuforia;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;


public class Supernova11567Vuforia {


    VuforiaLocalizer.Parameters parameters;
    VuforiaLocalizer vuforia;
    VuforiaTrackables beacons;

    Telemetry telemetry;

    boolean isLoopActive = false;

    /* constructor */
    public Supernova11567Vuforia(VuforiaLocalizer.CameraDirection cameraDirection, VuforiaLocalizer.Parameters.CameraMonitorFeedback feedBack, Telemetry telemetry) {

        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = cameraDirection;
        parameters.cameraMonitorFeedback = feedBack;
        parameters.vuforiaLicenseKey = " license key ";

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        beacons = vuforia.loadTrackablesFromAsset("UltimateGoal");
        beacons.get(0).setName("BlueTowerGoal");
        beacons.get(1).setName("RedTowerGoal");
        beacons.get(2).setName("RedAlliance");
        beacons.get(3).setName("BlueAlliance");
        beacons.get(4).setName("FrontWall");

        this.telemetry = telemetry;

    }

    public void init() {
        beacons.activate();
    }

    public void configureLoop(boolean isLoopActive) {
        this.isLoopActive = isLoopActive;
        loop();
    }

    public void loop() {
        while (isLoopActive) {
            for (VuforiaTrackable beacon : beacons) {
                OpenGLMatrix position = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();

                if (position != null) {

                    VectorF translation = position.getTranslation();
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beacon.getName() + "-translation:", String.valueOf(translation));
                    telemetry.addData(beacon.getName() + "-deegres:", String.valueOf(degreesToTurn));
                } else {

                }

            }

        }
    }

}
