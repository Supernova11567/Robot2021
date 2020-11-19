package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.RobotMotorsSetup;
import org.firstinspires.ftc.teamcode.RobotSensorsSetup;

@Autonomous(name = "Supernova11567Autonomous", group = "Supernova11567")

public class Supernova11567Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private org.firstinspires.ftc.teamcode.RobotMotorsSetup RobotMotorsSetup = new RobotMotorsSetup(hardwareMap, gamepad1, gamepad2);
    private RobotSensorsSetup RobotSensorsSetup = new RobotSensorsSetup(hardwareMap);
    private PIDautonomous pidAutonomous = new PIDautonomous(1, 1, 1);
    private Supernova11567Vuforia Supernova11567Vufofria = new Supernova11567Vuforia(VuforiaLocalizer.CameraDirection.BACK, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, telemetry);
    /* motors configuration on robot:
                                           /|\ forward
                                         1     0
                                         3     2         */
    //robot variables
    double yDistanceBetweenWheelsCenters = 10; //template_inch
    double xDistanceBetweenWheelsCenters = 10; //template_inch
    double robotCircleDiameter = yDistanceBetweenWheelsCenters / (Math.sin(Math.atan(yDistanceBetweenWheelsCenters / xDistanceBetweenWheelsCenters)));

    double wheelsDiameter = 4; //template_inch
    double ticksPerRotation_wheels = 1000; //template
    double inchesPerTick = (wheelsDiameter * Math.PI) / ticksPerRotation_wheels;

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

        moveWheelsByDistance(0, 60, true);
        //PID- robot needs to move forward 60 inch, track and set number of rings

        rotateRobotByAngle(40, true); //robot rotate right and tries to track red alliance wall image

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
                                    Supernova11567Vufofria.getBeaconPositionByName("RedAlliance").getTranslation()), true); //align robot to be vertical to the wall

                    while (Supernova11567Vufofria.getBeaconPositionByName("RedAlliance") == null) { //robot moves left until track image
                        moveWheelsManually(-90, 0.5);
                    }

                    moveWheelsByDistance(-90, 0.375 - Supernova11567Vufofria.getBeaconPositionByName("RedAlliance").getTranslation().get(0), true); //robot centers itself to the cube (centers the camera...)

                    pidAutonomous.resetAllCalculations();
                    pidAutonomous.PID_start(RobotMotorsSetup.w0.getCurrentPosition(), runtime.time(),
                            RobotSensorsSetup.distanceSensor.getDistance(DistanceUnit.INCH), 1);
                    while (pidAutonomous.reachedTarget == false) {
                        pidAutonomous.PID_calculate_byError(RobotSensorsSetup.distanceSensor.getDistance(DistanceUnit.INCH), runtime.time());
                    }
                    RobotMotorsSetup.stopAllMovement();

                    //puts wobble
            }

        } else {//left side full autonomous

        }
    }

    public void moveWheelsByDistance(double angle, double distanceToMove, boolean stopMovementAtEnd) {
        pidAutonomous.resetAllCalculations();
        RobotMotorsSetup.w0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotMotorsSetup.w1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotMotorsSetup.w2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotMotorsSetup.w3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RobotMotorsSetup.w0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotMotorsSetup.w1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotMotorsSetup.w2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotMotorsSetup.w3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidAutonomous.PID_start(0, runtime.time(), distanceToMove, 1);
        double movedDistance = 0;

        while (pidAutonomous.reachedTarget == false) {
            if (angle >= 0 && angle <= 90) {
                movedDistance = (Math.abs(RobotMotorsSetup.w1.getCurrentPosition()) * inchesPerTick) / Math.cos(angle);
            } else if (angle < 0 && angle >= -90) {
                movedDistance = (Math.abs(RobotMotorsSetup.w0.getCurrentPosition()) * inchesPerTick) / Math.cos(angle);
            } else if (angle < -90 && angle > -180) {
                movedDistance = (Math.abs(RobotMotorsSetup.w1.getCurrentPosition()) * inchesPerTick) / Math.cos(angle);
            } else if (angle > 90 && angle <= 180) {
                movedDistance = (Math.abs(RobotMotorsSetup.w0.getCurrentPosition()) * inchesPerTick) / Math.cos(angle);
            }

            RobotMotorsSetup.w0.setPower((-RobotMotorsSetup.getJoystickYValue(angle) - RobotMotorsSetup.getJoystickXValue(angle)) * pidAutonomous.PID_calculate(movedDistance, runtime.time()));
            RobotMotorsSetup.w1.setPower(-(RobotMotorsSetup.getJoystickYValue(angle) - RobotMotorsSetup.getJoystickXValue(angle)) * pidAutonomous.PID_calculate(movedDistance, runtime.time()));
            RobotMotorsSetup.w2.setPower((-RobotMotorsSetup.getJoystickYValue(angle) + RobotMotorsSetup.getJoystickXValue(angle)) * pidAutonomous.PID_calculate(movedDistance, runtime.time()));
            RobotMotorsSetup.w3.setPower(-(RobotMotorsSetup.getJoystickYValue(angle) + RobotMotorsSetup.getJoystickXValue(angle)) * pidAutonomous.PID_calculate(movedDistance, runtime.time()));
        }

        if (stopMovementAtEnd) {
            RobotMotorsSetup.stopAllMovement();
        }
    }

    public void moveWheelsManually(double angle, double speed) {
        RobotMotorsSetup.w0.setPower((-RobotMotorsSetup.getJoystickYValue(angle) - RobotMotorsSetup.getJoystickXValue(angle)) * speed);
        RobotMotorsSetup.w1.setPower(-(RobotMotorsSetup.getJoystickYValue(angle) - RobotMotorsSetup.getJoystickXValue(angle)) * speed);
        RobotMotorsSetup.w2.setPower((-RobotMotorsSetup.getJoystickYValue(angle) + RobotMotorsSetup.getJoystickXValue(angle)) * speed);
        RobotMotorsSetup.w3.setPower(-(RobotMotorsSetup.getJoystickYValue(angle) + RobotMotorsSetup.getJoystickXValue(angle)) * speed);
    }

    public void rotateRobotByAngle(double angles, boolean stopMovementAtEnd) {
        double distanceToRotate = (robotCircleDiameter * Math.PI) * (angles / 360);

        pidAutonomous.resetAllCalculations();
        pidAutonomous.PID_start(RobotMotorsSetup.w0.getCurrentPosition(), runtime.time(), distanceToRotate, 1);
        while (pidAutonomous.reachedTarget == false) {
            RobotMotorsSetup.w0.setPower(pidAutonomous.PID_calculate(RobotMotorsSetup.w0.getCurrentPosition(), runtime.time()));
            RobotMotorsSetup.w1.setPower(pidAutonomous.PID_calculate(RobotMotorsSetup.w0.getCurrentPosition(), runtime.time()));
            RobotMotorsSetup.w2.setPower(pidAutonomous.PID_calculate(RobotMotorsSetup.w0.getCurrentPosition(), runtime.time()));
            RobotMotorsSetup.w3.setPower(pidAutonomous.PID_calculate(RobotMotorsSetup.w0.getCurrentPosition(), runtime.time()));
        }

        if (stopMovementAtEnd) {
            RobotMotorsSetup.stopAllMovement();
        }
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