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
    private IntakeSystem IntakeSystem = new IntakeSystem(RobotMotorsSetup, gamepad1, gamepad2);

    //for display
    private double rightBrakePercentages = (gamepad1.right_trigger / 1) * 100;  //the percentages depend the maximum trigger value(1)
    private double leftBrakePercentages = (gamepad1.left_trigger / 1) * 100;  //the percentages depends the maximum trigger value(1)


    @Override
    public void init() {
        runtime.reset();

        RobotMotorsSetup.init();
        WheelsSystem.init();
        IntakeSystem.init();

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
        IntakeSystem.loop();
        
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

}