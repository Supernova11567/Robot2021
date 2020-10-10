package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Supernova11567Autonomous", group="Supernova11567Autonomous")

public class Supernova11567Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor_0 = hardwareMap.get(DcMotor.class, "motor_0");
    private DcMotor motor_1 = hardwareMap.get(DcMotor.class, "motor_1");
    private DcMotor motor_2 = hardwareMap.get(DcMotor.class, "motor_2");
    private DcMotor motor_3 = hardwareMap.get(DcMotor.class, "motor_3");
    /*
    motors configuration:      1     0
                               3     2
    */


    @Override
    public void runOpMode() {
        runtime.reset();


        waitForStart();

        
    }
}
