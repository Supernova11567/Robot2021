package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Supernova11567Autonomous", group="Supernova11567")

public class Supernova11567Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private RobotMotorsSetup RobotMotorsSetup = new RobotMotorsSetup();
    /* motors configuration on robot:
                                           /|\ forward
                                         1     0
                                         3     2         */


    @Override
    public void runOpMode() {

        //Until PLAY (after init)
        runtime.reset();
        RobotMotorsSetup.init(hardwareMap);

        waitForStart();

        //run once (after PLAY)


    }
}
