// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.BetterSwerveKinematics;

public class Constants {

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final int DRIVETRAIN_PIGEON_ID = 2; // Pigeon ID
    public static final int CANDLE_ID = 3;
    public static final String DRIVETRAIN_CANBUS = "canivore";

    public static final double DRIVE_SPEED = 1.0;
    public static final double BOOST_SPEED = 1.0;
    public static final double PERCISION_SPEED = 1.0;

    public static final class ModuleConstants {

        // Swerve Current Limiting
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
    
        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
    
        // Angle Motor PID Values
        public static final double angleKP = 40.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.5;
    
        // Drive Motor PID Values
        public static final double driveKP = 2.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
    
        // Drive Motor Characterization Values
        public static final double driveKS = 0.2;
        public static final double driveKV = 2.0;
    
        // Angle Encoder Invert
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;
    
        // Motor Inverts
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
    
        // Neutral Modes
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
    
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final double driveGearRatio = (50.0/14.0)*(17.0/27.0)*(45.0/15.0); //6.75:1
        public static final double angleGearRatio = (32.0/15.0)*(60.0/10.0); //12.8:1
        public static final double rotationsPerMeter = driveGearRatio / wheelCircumference;
    }

    public static final class SwerveConstants {
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(17.5);
        public static final double WHEELBASE_METERS = Units.inchesToMeters(26.5);
  
        public static final double MAX_VOLTAGE = 12.0;
  
        //public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)) * 0.10033 * Math.PI;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.0;
  
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);
  
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0), // Front Left
            new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0), // Front Right
            new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0), // Back Left
            new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0)); // Back Right

        public static final BetterSwerveKinematics BETTER_KINEMATICS = new BetterSwerveKinematics(
            new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0), // Front Left
            new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0), // Front Right
            new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0), // Back Left
            new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0)); // Back Right
  
        public static final int FRONT_LEFT_DRIVE_MOTOR = 19; // Front left module drive motor ID
        public static final int FRONT_LEFT_STEER_MOTOR = 20; // Front left module steer motor ID
        public static final int FRONT_LEFT_STEER_ENCODER = 21; // Front left steer encoder ID
        public static final double FRONT_LEFT_STEER_OFFSET = -0.3564453125; // Front left steer offset
  
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 14; // Front right drive motor ID
        public static final int FRONT_RIGHT_STEER_MOTOR = 13; // Front right steer motor ID
        public static final int FRONT_RIGHT_STEER_ENCODER = 15; // Front right steer encoder ID
        public static final double FRONT_RIGHT_STEER_OFFSET = -0.190673828125; // Front right steer offset
  
        public static final int BACK_LEFT_DRIVE_MOTOR = 16; // Back left drive motor ID
        public static final int BACK_LEFT_STEER_MOTOR = 17; // Back left steer motor ID
        public static final int BACK_LEFT_STEER_ENCODER = 18; // Back left steer encoder ID 
        public static final double BACK_LEFT_STEER_OFFSET = 0.2724609375; // Back left steer offset
  
        public static final int BACK_RIGHT_DRIVE_MOTOR = 10; // Back right drive motor ID
        public static final int BACK_RIGHT_STEER_MOTOR = 11; // Back right steer motor ID
        public static final int BACK_RIGHT_STEER_ENCODER = 12; // Back right steer encoder ID
        public static final double BACK_RIGHT_STEER_OFFSET = -0.01220703125; // Back right steer offset
    }
}
