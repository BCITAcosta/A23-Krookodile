// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.util.SwerveModuleConstants;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class NeoMotorConstants{
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class OperatorConstants {
    public static final int kOperatorJoyPort = 2;

    public static final int kL1Button = 0;
    public static final int kL2Button = 1;
    public static final int kL3Button = 2;
    public static final int kL4Button = 3;
    public static final int kSourceButton = 4;
    public static final int kProcessorButton = 5;
  }

  public static class DriverConstants{
    public static final int kDriverDriveJoy = 1;
    public static final int kDriverRotationJoy = 0;

    public static class DriveJoystickButtons{
      public static final int kIntakeButton = 1;
      public static final int kSlowModeButton = 2;
    }
    
    public static class RotationJoystickButtons{
      public static final int kOutakeButton = 1;
      public static final int kReturnTo0Degree = 2;
      public static final int k180DegreeFlip = 3;
      public static final int k60DegreeAutoLeft = 4;
      public static final int k60DegreeAutoRight = 5;

    }
  }

  public static class SwerveDriveConstants{
    public static final double kWheelBase = 24.229226;
    public static final double kTrackWidth = 24.229226;

    public static final double kBaseRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2))/2;

    public static final double kRealMaxSpeedMPS = 5.24256;
    public static final double kMaxAngularSpeed = 4 * Math.PI /3;
    public static final double kNormalModeTranslationSpeedScale = 1.0;
    public static final double kNormalModeRotationSpeedScale = 1.0;
    public static final double kSlowModeTranslationSpeedScale = 0.2;
    public static final double kSlowModeRotationSpeedScale = 0.2;

    public static final int kDrivingMotorPinionTeeth = 16;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // correction
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 Tooth Bevel Gear, 17 Tooth Bevel Gear Driving, 19 Tooth Second Stage Out, 
    // 27 Tooth Second Stage In, 16 Tooth Pinion, 50 Tooth First Stage
    public static final double kDrivingMotorReduction = (45.0 * 17 * 50) / (kDrivingMotorPinionTeeth * 15 * 27);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                    * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters
                    / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = (kWheelCircumferenceMeters
                    / kDrivingMotorReduction) / 60.0; // meters per second

    public static final Translation2d[] swerveModuleLocations = {
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // FL
      new Translation2d(kWheelBase / 2.0, -kTrackWidth/2.0), // FR
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // BL
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // BR
  };

      public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      swerveModuleLocations[0],
      swerveModuleLocations[1],
      swerveModuleLocations[2],
      swerveModuleLocations[3]);


      public static final boolean kUseRateLimit = true;
      public static final double kDirectionSlewRate = 4.5;
      public static final double kMagnitudeSlewRate = 4.5;
      public static final double kRotationalSlewRate = 2.0;


      public static final double kHeadingCorrectionP = 0.04;
      public static final double kHeadingCorrectionTolerance = 2.0;
}
  
  public static class Swerve{
    public static final class Mod0{
        public static final int kTurnSparkMaxID = 11;
        public static final int kDriveSparkMaxID = 12;
        public static final Rotation2d angleOffset = new Rotation2d(0.07);
        public static final boolean inverted = false;

        public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveSparkMaxID, kTurnSparkMaxID,
         angleOffset, inverted);
    }
    public static final class Mod1{
      public static final int kTurnSparkMaxID = 21;
      public static final int kDriveSparkMaxID = 22;
      public static final Rotation2d angleOffset = new Rotation2d(1.68);
      public static final boolean inverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveSparkMaxID, kTurnSparkMaxID,
       angleOffset, inverted);
    }
    public static final class Mod2{
      public static final int kTurnSparkMaxID = 31;
      public static final int kDriveSparkMaxID = 32;
      public static final Rotation2d angleOffset = new Rotation2d(0.26);
      public static final boolean inverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveSparkMaxID, kTurnSparkMaxID,
       angleOffset, inverted);
    }
    public static final class Mod3{
      public static final int kTurnSparkMaxID = 41;
      public static final int kDriveSparkMaxID = 42;
      public static final Rotation2d angleOffset = new Rotation2d(1.16);
      public static final boolean inverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveSparkMaxID, kTurnSparkMaxID,
       angleOffset, inverted);
    }
  }
}
