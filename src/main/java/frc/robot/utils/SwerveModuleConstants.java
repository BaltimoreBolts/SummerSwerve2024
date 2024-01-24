package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int thriftyEncoderID;
    public final Rotation2d thriftyOffsetDegrees;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int thriftyEncoderID, Rotation2d thriftyOffsetDegrees) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.thriftyEncoderID = thriftyEncoderID;
        this.thriftyOffsetDegrees = thriftyOffsetDegrees;
    }
}
