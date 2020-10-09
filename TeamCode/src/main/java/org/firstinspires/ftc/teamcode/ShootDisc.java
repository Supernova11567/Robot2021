package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class ShootDisc extends OpMode {

    double distance; //distance to the target
    double height;  //height of the target

    private Shooter shooter;  //creates new shooter object

    private PID pid = new PID(Constants.K_P, Constants.K_I, Constants.K_D); //PID constants


    public ShootDisc(double distance, double height, Shooter shooter) {

        this.distance = distance;
        this.height = height;
        this.shooter = shooter;

    }

    @Override
    public void init() {

        //uses the shooter class to calculate the velocity required and sets a target speed
        double V0 = shooter.CalcVelocity(distance, height);
        pid.setTarget(V0);

    }

    @Override
    public void loop() {
        //calculates the force for the motor
        double force = pid.OutputForce(shooter.getVelocityMotor(), shooter.getDeltaTime());

        shooter.setPower(force);

        shooter.periodic();
    }


}
