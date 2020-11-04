package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSystem {

    /* local OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private RobotMotorsSetup RobotMotorsSetup = null;
    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;


    /* Constructor */
    public IntakeSystem(RobotMotorsSetup RobotMotorsSetup, Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.RobotMotorsSetup = RobotMotorsSetup;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {

    }

    public void loop() {


    }

}

