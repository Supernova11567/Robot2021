package org.firstinspires.ftc.teamcode.Autonomous;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;


public class Vuforia {

    VuforiaLocalizer Vuforia;
    VuforiaLocalizer.Parameters parameters;

    /* Constructor */
    public Vuforia(VuforiaLocalizer.CameraDirection cameraDirection, VuforiaLocalizer.Parameters.CameraMonitorFeedback FeedBack) {

        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = cameraDirection;
        parameters.vuforiaLicenseKey = "license ley";
        parameters.cameraMonitorFeedback = FeedBack;

        Vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    }
    public void init() {
        
    }


}
