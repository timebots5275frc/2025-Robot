package frc.robot.CustomTypes;

public class SwerveCanIDs {
    public final int LEFT_FRONT_DRIVE_MOTOR_ID;
    public final int LEFT_FRONT_STEER_MOTOR_ID;
    public final int RIGHT_FRONT_DRIVE_MOTOR_ID;
    public final int RIGHT_FRONT_STEER_MOTOR_ID;
    public final int LEFT_REAR_DRIVE_MOTOR_ID;
    public final int LEFT_REAR_STEER_MOTOR_ID;
    public final int RIGHT_REAR_DRIVE_MOTOR_ID;
    public final int RIGHT_REAR_STEER_MOTOR_ID;

    public final int LEFT_FRONT_STEER_ENCODER_ID;
    public final int RIGHT_FRONT_STEER_ENCODER_ID;
    public final int LEFT_REAR_STEER_ENCODER_ID;
    public final int RIGHT_REAR_STEER_ENCODER_ID;

    public SwerveCanIDs(int LFDM, int LFSM, int RFDM, int RFSM, int LRDM, int LRSM, int RRDM, int RRSM, int LFSE, int RFSE, int LRSE, int RRSE)
    {
        LEFT_FRONT_DRIVE_MOTOR_ID = LFDM;
        LEFT_FRONT_STEER_MOTOR_ID = LFSM;
        RIGHT_FRONT_DRIVE_MOTOR_ID = RFDM;
        RIGHT_FRONT_STEER_MOTOR_ID = RFSM;
        LEFT_REAR_DRIVE_MOTOR_ID = LRDM;
        LEFT_REAR_STEER_MOTOR_ID = LRSM;
        RIGHT_REAR_DRIVE_MOTOR_ID = RRDM;
        RIGHT_REAR_STEER_MOTOR_ID = RRSM;

        LEFT_FRONT_STEER_ENCODER_ID = LFSE;
        RIGHT_FRONT_STEER_ENCODER_ID = RFSE;
        LEFT_REAR_STEER_ENCODER_ID = LRSE;
        RIGHT_REAR_STEER_ENCODER_ID = RRSE;
    }
}