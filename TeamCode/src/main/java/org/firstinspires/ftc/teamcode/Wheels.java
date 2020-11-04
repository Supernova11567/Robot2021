package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Wheels
{

    public DcMotor  w0   = null;
    public DcMotor  w1  = null;
    public DcMotor  w2   = null;
    public DcMotor  w3  = null;


    /* local OpMode members. */
    HardwareMap LocalHardwareMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Wheels(HardwareMap){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        LocalHardwareMap = hwMap;

        // Define and Initialize Motors
        w0  = hwMap.get(DcMotor.class, "w0");
        w1  = hwMap.get(DcMotor.class, "w1");
        w2  = hwMap.get(DcMotor.class, "w2");
        w3  = hwMap.get(DcMotor.class, "w3");

        w0.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        w1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        w2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        w3.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        w0.setPower(0);
        w1.setPower(0);
        w2.setPower(0);
        w3.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        w0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        w0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}

