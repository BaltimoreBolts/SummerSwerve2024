package frc.robot.utils;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int thriftyEncoderID;
    public final double thriftyOffsetDegrees;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int thriftyEncoderID, double thriftyOffsetDegrees) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.thriftyEncoderID = thriftyEncoderID;
        this.thriftyOffsetDegrees = thriftyOffsetDegrees;
    }
}
