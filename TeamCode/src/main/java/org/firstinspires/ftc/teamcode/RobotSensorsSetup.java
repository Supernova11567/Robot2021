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
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public RobotSensorsSetup(HardwareMap hwMap) {
        HardwareMap = hwMap;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
        // Define and Initialize Motors
        distanceSensor = HardwareMap.get(DistanceSensor.class, "distanceSensor");

    }

    public void loop() {

    }
}
