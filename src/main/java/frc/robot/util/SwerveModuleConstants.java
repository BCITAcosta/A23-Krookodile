package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int turnMotorID;
    public final Rotation2d angleOffset;
    public final boolean inverted;
    
        /**
         * Swerve Module Constants to be used when creating swerve modules;
         * @param driveMotorID
         * @param turnMotorID
         * @param angleOffset
         * @param inverted
     */
    public SwerveModuleConstants(int driveMotorID, int turnMotorID, Rotation2d angleOffset, boolean inverted){
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.angleOffset = angleOffset;
        this.inverted = inverted;
    }
}
