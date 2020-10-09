package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Supernova11567Main", group = "supernova11567")

public class Supernova11567Main extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor w0 = null;
    private DcMotor w1 = null;
    private DcMotor w2 = null;
    private DcMotor w3 = null;

    private double rightBrake = 0;
    private double leftBrake = 0;

    private double wheelDiameter = 10;  //the value is template...

    private double ticksPerRotation = 1000;  //the value is template...
    private double ticksPerInch = ticksPerRotation / (wheelDiameter * Math.PI);


    //for display
    private double rightBrakePercentages = (gamepad1.right_trigger / 1) * 100;  //the percentages depend the maximum trigger value(1)
    private double leftBrakePercentages = (gamepad1.left_trigger / 1) * 100;  //the percentages depends the maximum trigger value(1)
    private double MPS = 0;  //the average speed of the motors (meters per second)


    @Override
    public void init() {
        DcMotor w0 = hardwareMap.get(DcMotor.class, "motor_0");
        DcMotor w1 = hardwareMap.get(DcMotor.class, "motor_1");
        DcMotor w2 = hardwareMap.get(DcMotor.class, "motor_2");
        DcMotor w3 = hardwareMap.get(DcMotor.class, "motor_3");
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
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
    }


    @Override
    public void loop() {

        //brake
        rightBrake = -gamepad1.right_trigger + 1;
        leftBrake = -gamepad1.left_trigger + 1;


        //final move
        w0.setPower((-gamepad1.right_stick_y - gamepad1.right_stick_x) * rightBrake);
        w1.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) * leftBrake);
        w2.setPower((-gamepad1.right_stick_y + gamepad1.right_stick_x) * rightBrake);
        w3.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) * leftBrake);


        //display for drivers
        MPS = MPS();
        rightBrakePercentages = (gamepad1.right_trigger / 1) * 100;
        leftBrakePercentages = (gamepad1.left_trigger / 1) * 100;

        telemetry.addData("meters per second: ", MPS);
        telemetry.addData("rightBrake: ", rightBrakePercentages + "%");
        telemetry.addData("leftBrake: ", leftBrakePercentages + "%");
        telemetry.update();
    }


    @Override
    public void stop() {
    }


    public double MPS() {
        double w0PositionBeforeCalculate = w0.getCurrentPosition();
        double w1PositionBeforeCalculate = w1.getCurrentPosition();
        double w2PositionBeforeCalculate = w2.getCurrentPosition();
        double w3PositionBeforeCalculate = w3.getCurrentPosition();

        double runTimeStart = getRuntime();

        while ((runTimeStart + 1) != getRuntime()) {
            telemetry.addData("waiting until: ", runTimeStart + 1 - getRuntime());
            telemetry.update();
        }

        double w0CalculatedWay = Math.abs(w0.getCurrentPosition() - w0PositionBeforeCalculate) / ticksPerInch * 2.54 / 100;
        double w1CalculatedWay = Math.abs(w1.getCurrentPosition() - w1PositionBeforeCalculate) / ticksPerInch * 2.54 / 100;
        double w2CalculatedWay = Math.abs(w2.getCurrentPosition() - w2PositionBeforeCalculate) / ticksPerInch * 2.54 / 100;
        double w3CalculatedWay = Math.abs(w3.getCurrentPosition() - w3PositionBeforeCalculate) / ticksPerInch * 2.54 / 100;

        double motorsAverageWay = (w0CalculatedWay + w1CalculatedWay + w2CalculatedWay + w3CalculatedWay) / 4;

        return motorsAverageWay;
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
}