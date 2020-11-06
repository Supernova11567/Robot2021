package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class WheelsSystem {

    /* local OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private RobotMotorsSetup RobotMotorsSetup = null;
    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;

    private double rightBrake = 1;
    private double leftBrake = 1;

    private double wheelDiameter = 10;  //the value is template...

    private double ticksPerRotation = 1000;  //the value is template...
    private double ticksPerInch = ticksPerRotation / (wheelDiameter * Math.PI);


    /* Constructor */
    public WheelsSystem(RobotMotorsSetup RobotMotorsSetup, Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.RobotMotorsSetup = RobotMotorsSetup;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {

    }

    public void loop() {
        //brake
        rightBrake = -gamepad1.right_trigger + 1;
        leftBrake = -gamepad1.left_trigger + 1;

        RobotMotorsSetup.moveWheelsMecanumByJoysticks(rightBrake, leftBrake);
    }

}

