package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotMotorsSetup
{

    public DcMotor  w0   = null;
    public DcMotor  w1  = null;
    public DcMotor  w2   = null;
    public DcMotor  w3  = null;


    /* local OpMode members. */
    HardwareMap HardwareMap = null;
    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RobotMotorsSetup(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2){
        HardwareMap = hwMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
        // Save reference to Hardware map


        // Define and Initialize Motors
        w0  = HardwareMap.get(DcMotor.class, "w0");
        w1  = HardwareMap.get(DcMotor.class, "w1");
        w2  = HardwareMap.get(DcMotor.class, "w2");
        w3  = HardwareMap.get(DcMotor.class, "w3");

        w0.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        w1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        w2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        w3.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        w0.setPower(0);
        w1.setPower(0);
        w2.setPower(0);
        w3.setPower(0);

        w0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        w0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void loop() {

    }

    public void moveWheelsByMecanum (double rightBrake, double leftBrake) {
        w0.setPower((-gamepad1.right_stick_y - gamepad1.right_stick_x) * rightBrake);
        w1.setPower(-(gamepad1.left_stick_y - gamepad1.left_stick_x) * leftBrake);
        w2.setPower((-gamepad1.right_stick_y + gamepad1.right_stick_x) * rightBrake);
        w3.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x) * leftBrake);
    }
}

