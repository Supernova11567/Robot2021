package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Supernova11567Autonomous", group="Supernova11567")

public class Supernova11567Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor w0 = hardwareMap.get(DcMotor.class, "motor_0");
    private DcMotor w1 = hardwareMap.get(DcMotor.class, "motor_1");
    private DcMotor w2 = hardwareMap.get(DcMotor.class, "motor_2");
    private DcMotor w3 = hardwareMap.get(DcMotor.class, "motor_3");
    /* motors configuration on robot:
                                           /|\ forward
                                         1     0
                                         3     2         */


    @Override
    public void runOpMode() {

        //Until PLAY (after init)
        runtime.reset();

        w0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        w0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        w1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        w2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        w3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        w0.setDirection(DcMotorSimple.Direction.FORWARD);
        w1.setDirection(DcMotorSimple.Direction.FORWARD);
        w2.setDirection(DcMotorSimple.Direction.FORWARD);
        w3.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        //run once (after PLAY)


    }
}
