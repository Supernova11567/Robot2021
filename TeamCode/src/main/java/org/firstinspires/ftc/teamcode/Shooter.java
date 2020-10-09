package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.g;

public class Shooter {

    private DcMotor shootMotor;  //sets the first Dc motor to a null

    private double LastTime = 0;  //Last time measured
    private int LastPosition;  //Last position of motor

    private ElapsedTime runtime = new ElapsedTime();

    public Shooter() {

        //creates the 2 motor required for the shooter
        shootMotor  = hardwareMap.get(DcMotor.class, "shoot");

        //sets the direction that the motors will spin in
        shootMotor.setDirection(DcMotor.Direction.FORWARD);

        //sets the last position of the motor to the current position
        this.LastPosition = shootMotor.getCurrentPosition();

        //resets and starts the timer
        runtime.reset();
        runtime.startTime();
    }


    public void periodic() {
        
        //sets the last position to the current position and the last time to the current time
        this.LastPosition = shootMotor.getCurrentPosition();
        this.LastTime = runtime.seconds();
    }

    public void setPower(double power) {
        shootMotor.setPower(power);
    }

    //velocity of motor 1
    public double getVelocityMotor() {

        //calculates the change in time
        double seconds = getDeltaTime();

        //calculates the ticks and then position of the motor
        int ticks = shootMotor.getCurrentPosition() - this.LastPosition;
        double position = ticks/ Constants.TICKS_PER_METER;

        //calculates the velocity
        double velocity = position/seconds;

        return velocity; //self explanatory
    }

    public double getDeltaTime() {
        return runtime.seconds() - this.LastTime;
    }

    /**
     * calculates the necessary speed
     * @param d the distance to the target [m].
     * @param h the relative height of the target [m].
     * @return the optimum velocity to reach the target at angle 0 [m/s].
     */
    public double CalcVelocity(double d, double h) {

        double velocity = Math.sqrt((d*d*g)/(2*h)+(h*2*g));

        return velocity;
    }

    /**
     * calculates the necessary speed
     * @param d is the distance to the target [m].
     * @param h is the relative height to the target [m].
     * @return the optimum angle required to reach the target at angle 0 [degrees].
     */
    public double CalcAngle(double d, double h) {

        double angle = Math.atan((2*h)/d);

        return angle;
    }
}
