package frc.robot.CustomTypes;


// These constants define the location of the wheels from the center of the robot.
// These coordinates are determined by the right hand rule.
// Index finger points in the forward X direction, Thumb points up in the positive Z direction, Middle finger points left in the positive Y direction.

public class SwerveModuleLocations {
    // IN METERS
    public final double LEFT_FRONT_WHEEL_X;
    public final double LEFT_FRONT_WHEEL_Y;
    public final double RIGHT_FRONT_WHEEL_X;
    public final double RIGHT_FRONT_WHEEL_Y;
    public final double RIGHT_REAR_WHEEL_X;
    public final double RIGHT_REAR_WHEEL_Y;
    public final double LEFT_REAR_WHEEL_X;
    public final double LEFT_REAR_WHEEL_Y;

    public SwerveModuleLocations(double LFWX, double LFWY, double RFWX, double RFWY, double RRWX, double RRWY, double LRWX, double LRWY)
    {
        LEFT_FRONT_WHEEL_X = LFWX;
        LEFT_FRONT_WHEEL_Y = LFWY;
        RIGHT_FRONT_WHEEL_X = RFWX;
        RIGHT_FRONT_WHEEL_Y = RFWY;
        RIGHT_REAR_WHEEL_X = RRWX;
        RIGHT_REAR_WHEEL_Y = RRWY;
        LEFT_REAR_WHEEL_X = LRWX;
        LEFT_REAR_WHEEL_Y = LRWY;
    }
}