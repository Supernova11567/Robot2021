package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RobotMotorsSetup;

@Autonomous(name = "Supernova11567Autonomous", group = "Supernova11567")

public class Supernova11567Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private org.firstinspires.ftc.teamcode.RobotMotorsSetup RobotMotorsSetup = new RobotMotorsSetup(hardwareMap, gamepad1, gamepad2);
    private Supernova11567Vuforia Supernova11567Vufofria = new Supernova11567Vuforia(VuforiaLocalizer.CameraDirection.BACK, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, );
    /* motors configuration on robot:
                                           /|\ forward
                                         1     0
                                         3     2         */

    @Override
    public void runOpMode() {

        //Until PLAY (after init)
        runtime.reset();
        RobotMotorsSetup.init();

        waitForStart();

        //run once (after PLAY)


    }

    public void addTelemetryData (String title, String data) {
        telemetry.addData(title, data);
    }
}
