package org.firstinspires.ftc.teamcode;

public class Helpers {
    /*
    * PORTS
    * Motor:
    *   0 - intake
    *   1
    *   2
    *   3
    *   4
    * */

    public static double slew(double SLEW_RATE, double input, double prev) {
        if (SLEW_RATE < Math.abs(input - prev)) {
            if (input < prev) {
                return prev - SLEW_RATE;
            } else if (input > prev) {
                return prev + SLEW_RATE;
            }
        }
        return input;
    }
}
