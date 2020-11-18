package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDautonomous {

    PIDCoefficients pidCoefficients;

    double startPosition = 0;
    double startTime = 0;


    /* constructor */
    public PIDautonomous (double kp, double ki, double kd) {
        pidCoefficients = new PIDCoefficients(kp, ki, kd);
    }

    public void resetAllCalculations () {

    }

    public double PID_start (double startPosition, double starTime) {
        
        this.startPosition = startPosition;
        this.startTime = starTime;
    }
}
