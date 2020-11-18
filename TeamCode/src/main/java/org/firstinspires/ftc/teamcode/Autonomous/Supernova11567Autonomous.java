package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.RobotMotorsSetup;
import org.firstinspires.ftc.teamcode.RobotSensorsSetup;

@Autonomous(name = "Supernova11567Autonomous", group = "Supernova11567")

public class Supernova11567Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private org.firstinspires.ftc.teamcode.RobotMotorsSetup RobotMotorsSetup = new RobotMotorsSetup(hardwareMap, gamepad1, gamepad2);
    private RobotSensorsSetup RobotSensorsSetup = new RobotSensorsSetup(hardwareMap, gamepad1, gamepad2);
    private PIDautonomous pidAutonomous = new PIDautonomous(1, 1, 1);
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

        Supernova11567Vufofria.configureLoop(true); //vuforia starts loop (track images)

        //robot starts vertical to wall- and wobble doesn't in front of the robot (to not disturb)
        //and phone must start vertical                                                                         !!!

        //PID- robot needs to move forward 60 inch, track and set number of rings

        rotateRobotByAngle(40); //robot rotate right and tries to track red alliance wall image

        if (((VuforiaTrackableDefaultListener) Supernova11567Vufofria.beacons.get(2).getListener()).getPose() == null) {
            //didn't track right side target image
            startedRight = false;
        } else {
            startedRight = true;
        }


        if (startedRight) { //right side full autonomous

            switch (numberOfStartedRings) { //putting wobble at the correct square
                case 0:
                    rotateRobotByAngle(
                            Supernova11567Vufofria.degreesToAlignImageTranslation(
                                    Supernova11567Vufofria.getBeaconPositionByName("RedAlliance").getTranslation())); //align robot to be vertical to the wall

                    while (Supernova11567Vufofria.getBeaconPositionByName("RedAlliance") == null) { //robot moves left until track image
                        moveWheelsManually(-90, 0.2);
                    }

                    //      PID_moveLeft( 0.375-Supernova11567Vufofria.getBeaconPositionByName("RedAlliance").getTranslation().get(0) ) //robot centers itself to the cube (centers the camera...)
            }

        } else {//left side full autonomous

        }
    }

    public void moveWheelsManually(double angle, double speed) {
        RobotMotorsSetup.w0.setPower((-RobotMotorsSetup.getJoystickYValue(angle) - RobotMotorsSetup.getJoystickXValue(angle)) * speed);
        RobotMotorsSetup.w1.setPower(-(RobotMotorsSetup.getJoystickYValue(angle) - RobotMotorsSetup.getJoystickXValue(angle)) * speed);
        RobotMotorsSetup.w2.setPower((-RobotMotorsSetup.getJoystickYValue(angle) + RobotMotorsSetup.getJoystickXValue(angle)) * speed);
        RobotMotorsSetup.w3.setPower(-(RobotMotorsSetup.getJoystickYValue(angle) + RobotMotorsSetup.getJoystickXValue(angle)) * speed);
    }

    public void rotateRobotByAngle(double angles) {

    }

    public void rotateRobotManually(boolean clockwise, double speed) {
        if (clockwise) {
            RobotMotorsSetup.w0.setPower(-speed);
            RobotMotorsSetup.w1.setPower(-speed);
            RobotMotorsSetup.w2.setPower(-speed);
            RobotMotorsSetup.w3.setPower(-speed);
        } else {
            RobotMotorsSetup.w0.setPower(speed);
            RobotMotorsSetup.w1.setPower(speed);
            RobotMotorsSetup.w2.setPower(speed);
            RobotMotorsSetup.w3.setPower(speed);
        }
    }

}
