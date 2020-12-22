package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PID {

    private PIDCoefficients pid;
    private double target;
    private double lastError = 0;
    private double integral = 0;

    public PID(double k_p, double k_i, double k_d) {
        pid = new PIDCoefficients(k_p, k_i, k_d);
    }

    //sets the target of the pid
    public void setTarget(double target) {
        this.target = target;
    }

    public double OutputForce(double input, double deltaTime) {
        double error = target - input;

        double p = pid.p * error;

        integral += error * deltaTime;
        double i = pid.i * integral;

        double delta_e = lastError - error;
        double derivative = delta_e / deltaTime;
        double d = pid.d * derivative;

        double force = p + i + d;

        lastError = error;
        return force;
    }
}
