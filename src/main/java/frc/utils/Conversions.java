package frc.utils;

public class Conversions {
    public static double encoder_ticks_to_rpm(double tickrate, double gearratio) {
        double motorRPM = tickrate * (600.0/2048.0);
        return motorRPM / gearratio;
    }

    public static double RPMtoMPS(double RPM, double circumference) {
        return (RPM * circumference) / 60;
    }
}
