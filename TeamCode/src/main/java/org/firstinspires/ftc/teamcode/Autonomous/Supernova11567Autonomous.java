package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RobotMotorsSetup;

@Autonomous(name = "Supernova11567Autonomous", group = "Supernova11567")

public class Supernova11567Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private org.firstinspires.ftc.teamcode.RobotMotorsSetup RobotMotorsSetup = new RobotMotorsSetup(hardwareMap, gamepad1, gamepad2);
    private Supernova11567Vuforia Supernova11567Vufofria = new Supernova11567Vuforia(VuforiaLocalizer.CameraDirection.BACK, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, telemetry);
    /* motors configuration on robot:
                                           /|\ forward
                                         1     0
                                         3     2         */

    //autonomous variables
    boolean startedRight = Boolean.parseBoolean(null);
    int numberOfStartedRings;

    @Override
    public void runOpMode() {

        //Until PLAY (after init)
        runtime.reset();
        RobotMotorsSetup.init();
        Supernova11567Vufofria.init();

        waitForStart();
        //run once (after PLAY)

        Supernova11567Vufofria.configureLoop(true); //vuforia starts loop

        //robot starts vertical to wall- and wobble doesn't in front of the robot (to not disturb)
        while (startedRight = Boolean.parseBoolean(null)) {
            double angle = 0;
            RobotMotorsSetup.moveWheelsManually(angle,0.25);

        }

    }
}
