package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Supernova11567Main", group = "supernova11567")

public class Supernova11567Main extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private RobotMotorsSetup RobotMotorsSetup = new RobotMotorsSetup(hardwareMap, gamepad1, gamepad2);
    private org.firstinspires.ftc.teamcode.WheelsSystem WheelsSystem = new WheelsSystem(RobotMotorsSetup, gamepad1, gamepad2);

    //for display
    private double rightBrakePercentages = (gamepad1.right_trigger / 1) * 100;  //the percentages depend the maximum trigger value(1)
    private double leftBrakePercentages = (gamepad1.left_trigger / 1) * 100;  //the percentages depends the maximum trigger value(1)


    @Override
    public void init() {
        runtime.reset();

        RobotMotorsSetup.init();
        WheelsSystem.init();

    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {

    }


    @Override
    public void loop() {

        RobotMotorsSetup.loop();
        WheelsSystem.loop();

        //display for drivers
        rightBrakePercentages = (gamepad1.right_trigger / 1) * 100;
        leftBrakePercentages = (gamepad1.left_trigger / 1) * 100;

        telemetry.addData("rightBrake: ", rightBrakePercentages + "%");
        telemetry.addData("leftBrake: ", leftBrakePercentages + "%");
        telemetry.update();
    }


    @Override
    public void stop() {
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