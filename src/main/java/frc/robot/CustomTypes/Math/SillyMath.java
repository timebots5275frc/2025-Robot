package frc.robot.CustomTypes.Math;

public class SillyMath {
    public static double clamp(double in, double min, double max)
    {
        if (in > max) { return max; }
        else if (in < min) { return min; }

        return in;
    }

    public static double goodSqrt(double in)
    {
        if (in >= 0) { return Math.sqrt(in); }
        return -Math.sqrt(-in);
    }
}