package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static final class ModuleConstants{
    public static final double WheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/8.14;
    public static final double kTurningMotorGearRatio = 1/21.43;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * WheelDiameterMeters; // converts rotation to meters
    public static final double kTurningEncoderRot2Meter = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2MeterPerSec = kTurningEncoderRot2Meter / 60;
    //public static final double kPTurning = 0.5; //PID -> P Value for turning
  }

  public static final class DriveConstants{
    public static final double kWidth = Units.inchesToMeters(25); //Length between right & left wheels, essentially robot width
    public static final double kLength = Units.inchesToMeters(25); //Length between front & back wheels, essentially robot length
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(kLength / 2, -kWidth / 2), 
                                                                                          new Translation2d(kLength / 2, kWidth / 2), 
                                                                                          new Translation2d(-kLength / 2, -kWidth / 2),
                                                                                          new Translation2d(-kLength / 2, kWidth / 2));
    public static final int frontLeftDriveMotorPort = 11;
    public static final int frontLeftTurningMotorPort = 10;
    public static final int frontRightDriveMotorPort = 2;
    public static final int frontRightTurningMotorPort = 1;
    public static final int backLeftDriveMotorPort = 8;
    public static final int backLeftTurningMotorPort = 7;
    public static final int backRightDriveMotorPort = 5;
    public static final int backRightTurningMotorPort = 4;

    public static final int frontLeftAbsoluteEncoder = 3;
    public static final int frontRightAbsoluteEncoder = 6;
    public static final int backLeftAbsoluteEncoder = 9;
    public static final int backRightAbsoluteEncoder = 12;

    public static final double physicalMaxSpeedMetersPerSecond = 10;
    public static final double physicalMaxAngularSpeedRadiansPerSecond = 2*2*Math.PI;
    public static final double teleDriveMaxSpeedMetersPerSecond = physicalMaxSpeedMetersPerSecond/4;
    public static final double teleDriveMaxAngularSpeedRadiansPerSecond = physicalMaxAngularSpeedRadiansPerSecond/4;
    public static final double teleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double teleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    
  }
  public static final class OIConstants{
    public static final int DriveControllerPort = 0;

    public static final int DriverYAxis = 1;
    public static final int DriverXAxis = 0;
    public static final int kDriverRotAxis = 2;
    public static final int DriverFieldOrientedButton = 1;

    public static final double deadband = 0.001;
  }
}