package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDautonomous {

    PIDCoefficients pidCoefficients;


    /* constructor */
    public PIDautonomous (double kp, double ki, double kd) {
        pidCoefficients = new PIDCoefficients(kp, ki, kd);
    }
    

}
