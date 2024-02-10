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
    public static final String DRIVETRAIN_CANBUS = "CANivore1";

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
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;

        // Neutral Modes
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        // billet wheels have a 4" OD as per spec, measuered distance in onshape
        // is 3.95" with wear diammeter will be reduced
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        // MK4i drive gear ratios
        // L1 L2(using) L3
        // stage Driving Driven Driving Driven Driving Driven
        // 1 14 50 14 50 14 50
        // 2 25 19 27 17 28 16
        // 3 15 45 15 45 15 45
        // 8.14:1 6.75:1 6.12:1
        private static final double Stage1Ratio = 50.0 / 14.0;
        private static final double Stage2Ratio = 17.0 / 27.0;
        private static final double Stage3Ratio = 45.0 / 15.0;
        public static final double driveGearRatio = Stage1Ratio * Stage2Ratio * Stage3Ratio; // 6.75:1

        // The steering gear ratio of the MK4i is 150/7:1
        public static final double angleGearRatio = 21.43;

        // The below line of code has the default angle gear ratio from the repo.
        // I kept because it's significantly different from the ratio listed on SDS's
        // website.
        // I'm not sure why they are so different.
        // public static final double angleGearRatio = (32.0/15.0)*(60.0/10.0); //12.8:1

        public static final double rotationsPerMeter = driveGearRatio / wheelCircumference;
    }

    public static final class SwerveConstants {
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(26.75);
        public static final double WHEELBASE_METERS = Units.inchesToMeters(22.25);

        public static final double MAX_VOLTAGE = 12.0;

        // MK4I speeds in ft/s using falcon 500 L1: 13.7 L2: 16.5 L3: 18.2
        // public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        // ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)) * 0.100076 * Math.PI;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = Units.feetToMeters(16.5);

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

        /*
         * +X
         * |
         * |
         * |
         * +Y | -Y
         * <------------|---------->
         * |
         * |
         * |
         * |
         * -X
         */

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0), // Front Left
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0), // Front Right
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0), // Back Left
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)); // Back Right

        public static final BetterSwerveKinematics BETTER_KINEMATICS = new BetterSwerveKinematics(
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0), // Front Left
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0), // Front Right
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0), // Back Left
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)); // Back Right

        public static final String FRONT_LEFT_MODULE_NAME = "FrontLeft";
        public static final int FRONT_LEFT_DRIVE_MOTOR = 20; // Front left module drive motor ID
        public static final int FRONT_LEFT_STEER_MOTOR = 22; // Front left module steer motor ID
        public static final int FRONT_LEFT_STEER_ENCODER = 53; // Front left steer encoder ID
        public static final double FRONT_LEFT_STEER_OFFSET = -0.1748046875; // Front left steer offset

        public static final String FRONT_RIGHT_MODULE_NAME = "FrontRight";
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 23; // Front right drive motor ID
        public static final int FRONT_RIGHT_STEER_MOTOR = 41; // Front right steer motor ID
        public static final int FRONT_RIGHT_STEER_ENCODER = 52; // Front right steer encoder ID
        public static final double FRONT_RIGHT_STEER_OFFSET = 0.41259765625; // Front right steer offset

        public static final String BACK_LEFT_MODULE_NAME = "BackLeft";
        public static final int BACK_LEFT_DRIVE_MOTOR = 43; // Back left drive motor ID
        public static final int BACK_LEFT_STEER_MOTOR = 44; // Back left steer motor ID
        public static final int BACK_LEFT_STEER_ENCODER = 51; // Back left steer encoder ID
        public static final double BACK_LEFT_STEER_OFFSET = 0.060302734375; // Back left steer offset

        public static final String BACK_RIGHT_MODULE_NAME = "BackRight";
        public static final int BACK_RIGHT_DRIVE_MOTOR = 45; // Back right drive motor ID
        public static final int BACK_RIGHT_STEER_MOTOR = 40; // Back right steer motor ID
        public static final int BACK_RIGHT_STEER_ENCODER = 50; // Back right steer encoder ID
        public static final double BACK_RIGHT_STEER_OFFSET = 0.106201171875; // Back right steer offset
    }

    public static final class PivotConstants {

        public static int SparkmaxDeviceID = 31;

        public static final class PivotPIDConstants {

            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kIz = 0;
            public static final double kFF = 0;
            public static final double kMaxOutput = 0;
            public static final double kMinOutput = 0;

            public static final double DumpPoint = 0;
            public static final double ShootingPoint = 0;
            public static final double StartingPoint = 0;

        }

        public static final class PivotPostions {
            public static final double Starting = 0;
            public static final double Dumping = 0;
            public static final double Shooting = 0;
        }

    }

    public static final class FiringHeadConstants {

        public static int UpperSparkmaxDeviceID = 9;
        public static int LowerSparkmaxDeviceID = 55;
        public static int UpperTransportSparkmaxDeviceID = 33;

        public static double FiringSpeed = 0;

        public static final class FiringHeadPIDConstants {
            
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kIz = 0;
            public static final double kFF = 0;
            public static final double kMaxOutput = 0;
            public static final double kMinOutput = 0;

            public static final double FireVelocity = 0;

        }
        

    }

    public static final class IntakeConstants {

        public static int LowerIntakeSparkmaxDeviceID = 11;
        public static int UpperConvyerSparkmaxDeviceID = 1;
        public static int LowerConvyerSparkmaxDeviceID = 2;
        public static int UpperIntakeSparkmaxDeviceID = 33;
        public static int TimeOfFlightSensorId = 0;
        public static double ballTreshholdIntake = 0;
        public static double IntakeMotorSpeed = 0;
        public static double ConveyrMotorSpeed = 0;

    }
}
