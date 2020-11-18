package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDautonomous {

    PIDCoefficients pidCoefficients;

    double startPosition = 0;
    double startTime = 0;
    double distanceToMove = 0;


    /* constructor */
    public PIDautonomous (double kp, double ki, double kd) {
        pidCoefficients = new PIDCoefficients(kp, ki, kd);
    }

    public void resetAllCalculations () {
        startPosition = 0;
        startTime = 0;
    }

    public void PID_start (double startPosition, double starTime, double DistanceToMove) {
        this.startPosition = startPosition;
        this.startTime = starTime;
        this.distanceToMove = distanceToMove;
    }

    public double PID_calculate (double currentPosition, double currentTime) {
        
    }
}
