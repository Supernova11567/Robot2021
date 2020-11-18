package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotSensorsSetup {
    public DistanceSensor distanceSensor = null;


    /* local OpMode members. */
    com.qualcomm.robotcore.hardware.HardwareMap HardwareMap = null;
    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RobotSensorsSetup(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2) {
        HardwareMap = hwMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
        // Define and Initialize Motors
        distanceSensor = HardwareMap.get(DistanceSensor.class, "distanceSensor");

    }

    public void loop() {

    }
}
