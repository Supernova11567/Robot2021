package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RobotMotorsSetup {

    public DcMotor w0 = null;
    public DcMotor w1 = null;
    public DcMotor w2 = null;
    public DcMotor w3 = null;
    public DcMotor motor_intake = null;


    /* local OpMode members. */
    HardwareMap HardwareMap = null;
    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public RobotMotorsSetup(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2) {
        HardwareMap = hwMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
        // Define and Initialize Motors
        w0 = HardwareMap.get(DcMotor.class, "w0");
        w1 = HardwareMap.get(DcMotor.class, "w1");
        w2 = HardwareMap.get(DcMotor.class, "w2");
        w3 = HardwareMap.get(DcMotor.class, "w3");
        motor_intake = HardwareMap.get(DcMotor.class, "motor_intake");

        w0.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        w1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        w2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        w3.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motor_intake.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        w0.setPower(0);
        w1.setPower(0);
        w2.setPower(0);
        w3.setPower(0);
        motor_intake.setPower(0);

        w0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        w3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        w0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void loop() {

    }

    public double getJoystickXValue(double angle) {
        double newAngleX = angle;
        double xValue;

        if (Math.abs(newAngleX) > 179) {
            newAngleX = 0;
        } else {
            newAngleX = Range.clip(newAngleX, -179, 179);
        }

        if (Math.abs(newAngleX) > 90) {
            xValue = ((Math.abs(newAngleX) / 90) - 2 * ((Math.abs(newAngleX) / 90) % 1)) * (newAngleX / Math.abs(newAngleX));
            return xValue;
        } else {
            xValue = newAngleX / 90;
            return xValue;
        }
    }

    public double getJoystickYValue(double angle) {
        double newAngleY = angle;
        double yValue;

        yValue = (Math.abs(newAngleY) / 90) - 1;

        return yValue;
    }

    public void moveWheelsMecanumByJoysticks(double rightBrake, double leftBrake) {
        w0.setPower((-gamepad1.right_stick_y - gamepad1.right_stick_x) * rightBrake);
        w1.setPower(-(gamepad1.left_stick_y - gamepad1.left_stick_x) * leftBrake);
        w2.setPower((-gamepad1.right_stick_y + gamepad1.right_stick_x) * rightBrake);
        w3.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x) * leftBrake);
    }

    public void moveIntake(double speed) {
        motor_intake.setPower(speed);
    }

    public void stopAllMovement () {
        //resets all motors to 0 speed
        w0.setPower(0);
        w1.setPower(0);
        w2.setPower(0);
        w3.setPower(0);
        motor_intake.setPower(0);
    }
}

