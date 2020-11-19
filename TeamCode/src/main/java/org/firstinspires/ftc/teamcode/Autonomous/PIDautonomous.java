package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDautonomous {

    PIDCoefficients pidCoefficients;

    double startPosition = 0;
    double startTime = 0;
    double distanceToMove = 0;

    //PID formula variables
    double error;
    double integral;
    double derivative;

    double lastError;
    double lastTime;

    double maximumErrorFromTarget;
    boolean reachedTarget;

    /* constructor */
    public PIDautonomous (double kp, double ki, double kd) {
        pidCoefficients = new PIDCoefficients(kp, ki, kd);
    }

    public void resetAllCalculations () {
        startPosition = 0;
        startTime = 0;
        distanceToMove = 0;

        error = 0;
        integral = 0;
        derivative = 0;

        lastError = 0;
        lastTime = 0;

        maximumErrorFromTarget = 0;
        reachedTarget = false;
    }

    public void PID_start (double startPosition, double starTime, double DistanceToMove, double maximumErrorFromTarget) {
        this.startPosition = startPosition;
        this.startTime = starTime;
        this.distanceToMove = distanceToMove;
        this.maximumErrorFromTarget = maximumErrorFromTarget;
    }

    public double PID_calculate (double currentPosition, double currentTime) {
        //P
        error = distanceToMove - (currentPosition - startPosition);

        if (error < maximumErrorFromTarget) {
            reachedTarget = true;
        }

        //I
        integral += currentPosition * (currentTime - lastTime);

        //D
        derivative = ( lastError - error ) / ( currentTime - lastTime );

        double finalOutput = pidCoefficients.p * error + pidCoefficients.i * integral + pidCoefficients.d * derivative;

        lastError = error;
        lastTime = currentTime;

        return finalOutput;
    }

    public double PID_calculate_byError (double error, double currentTime) {
        //P
        this.error = error;

        if (error < maximumErrorFromTarget) {
            reachedTarget = true;
        }

        //I
        integral += (distanceToMove - startPosition - error) * (currentTime - lastTime);

        //D
        derivative = ( lastError - error ) / ( currentTime - lastTime );

        double finalOutput = pidCoefficients.p * error + pidCoefficients.i * integral + pidCoefficients.d * derivative;

        lastError = error;
        lastTime = currentTime;

        return finalOutput;
    }
}
