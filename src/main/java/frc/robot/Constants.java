// Archivo donde se guardan las constantes del robot
////////////////////////////////////////////////////
// File where the robot constants are stored

package frc.robot;

import org.team3526.lib.util.Conversions;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public final static class Swerve {

    public static final class Module {
      public static final double kWheelDiameterMeters = Conversions.inchToM(4.0); // 4 inches
      public static final double kDriveMotorGearRatio = 1.0 / 6.12; // 6.12:1 Drive
      public static final double kTurningMotorGearRatio = 1.0 / 12.8; // 12.8:1 Steering

      public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI; // Conversion Rotaciones a Metros
      public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0; // Conversion RPM a Metros por Segundo

      public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI; // Conversion Rotaciones a Radianes
      public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0; // Conversion RPM a Radianes por Segundo

      public static final double kGlobalTurningOffsetRad = Math.toRadians(180);

      public static final class PIDParameters {
        public static double m_kP = 0.15;
        public static double m_kI = 0.0;
        public static double m_kD = 0.0;
      }

      private static final PIDController m_turningPIDController = new PIDController(Constants.Swerve.Module.PIDParameters.m_kP, Constants.Swerve.Module.PIDParameters.m_kI, Constants.Swerve.Module.PIDParameters.m_kD);
      public static final PIDController getTurningPIDController() {
        m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);
        return m_turningPIDController;
      }
    }

    public final static class Physical {
        //582.5 mm LEFT RIGHT
        //583.0 mm FRONT BACK
        public static final double kTrackWidth = Conversions.mmToM(582.5); // Distance between left and right wheels
        public static final double kWheelBase = Conversions.mmToM(583.0); // Distance between front and back wheels

        public static final Translation2d m_frontLeftLocation = new Translation2d(kWheelBase/2, -kTrackWidth/2); // Front Left Wheel Location
        public static final Translation2d m_frontRightLocation = new Translation2d(kWheelBase/2, kTrackWidth/2); // Front Right Wheel Location
        public static final Translation2d m_backLeftLocation = new Translation2d(-kWheelBase/2, -kTrackWidth/2); // Back Left Wheel Location
        public static final Translation2d m_backRightLocation = new Translation2d(-kWheelBase/2, kTrackWidth/2); // Back Right Wheel Location

        public static final double kMaxSpeedMetersPerSecond = 10.0; // Maxima Velocidad en Metros por Segundo
        //public static final double kMaxSpeedMetersPerSecond = 5.0; // Maxima Velocidad en Metros por Segundo
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * 2.0 * Math.PI; // Maxima Velocidad Angular en Radianes por Segundo

        public static final double kMaxAccelerationUnitsPerSecond = 3.0; // Maxima Aceleracion
        public static final double kMaxAngularAccelerationUnitsPerSecond = Math.PI / 4.0; // Maxima Aceleracion Angular

        public static final double kTeleopMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 4.0; // Maxima Velocidad en Metros por Segundo
        public static final double kTeleopMaxAngularSpeedRadiansPerSecond = kMaxAngularSpeedRadiansPerSecond / 4.0; // Maxima Velocidad Angular en Radianes por Segundo

        public static final double kTeleopMaxAccelerationUnitsPerSecond = 3.0; // Maxima Aceleracion
        public static final double kTeleopMaxAngularAccelerationUnitsPerSecond = 3.0; // Maxima Aceleracion Angular

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationUnitsPerSecond);

        public static final SwerveDriveKinematics m_swerveDriveKinematics = new SwerveDriveKinematics(
          Constants.Swerve.Physical.m_frontLeftLocation,
          Constants.Swerve.Physical.m_frontRightLocation,
          Constants.Swerve.Physical.m_backLeftLocation,
          Constants.Swerve.Physical.m_backRightLocation
        );


      }

      public static final class Autonomous {
        public static final PIDConstants kTranslatePIDConstants = new PIDConstants(0.1, 0.0, 0.0);
        public static final PIDConstants kRotatePIDConstants = new PIDConstants(0.1, 0.0, 0.0);

        public static final double kMaxSpeedMetersPerSecond = 1.0; // Maxima Velocidad en Metros por Segundo
      }

    public final static class Motors {
      //! OFFSETS ARE CALCULATED AS THE -(THE DIFFERENCE OF SWERVE ABSOLUTE ENCODER "0" and ABSOLUTE ENCODER 0 ---IN RADIANS---)
      public static final Object[] kFrontLeftVars = { 
        Math.toRadians(-107.841796875), // Offset
        true, // Inverted
        11, // Absolute Encoder ID
        22, // Drive Motor ID
        21, // Turning Motor ID
        false, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Front Left" // Name
      };
      public static final Object[] kFrontRightVars = {
        Math.toRadians(-38.583984375), // Offset
        true, // Inverted
        12, // Absolute Encoder ID
        24, // Drive Motor ID
        23, // Turning Motor ID
        false, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Front Right" // Name
      };
      public static final Object[] kBackLeftVars = {
        Math.toRadians(-56.953), // Offset
        true, // Inverted
        13, // Absolute Encoder ID
        26, // Drive Motor ID
        25, // Turning Motor ID
        true, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Back Left" // Name
      };
      public static final Object[] kBackRightVars = {
        Math.toRadians(-105.46875), // Offset
        true, // Inverted
        14, // Absolute Encoder ID
        28, // Turning Motor ID
        27, // Drive Motor ID
        false, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Back Right" // Name
      };
    }
  }

  public static final class Operator { // Operator Controllers and Data
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDeadzone = 0.1;
  }

}