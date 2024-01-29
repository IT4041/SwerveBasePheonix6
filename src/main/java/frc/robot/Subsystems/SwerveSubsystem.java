// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BetterSwerveKinematics;
import frc.lib.BetterSwerveModuleState;
import frc.lib.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveModule[] swerveModules;
  BaseStatusSignal[] signals;

  public SwerveSubsystem() {
    
    swerveModules = new SwerveModule[] {
      new SwerveModule(0, new SwerveModuleConstants(
      SwerveConstants.FRONT_LEFT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_ENCODER, 
      SwerveConstants.FRONT_LEFT_STEER_OFFSET)),

      new SwerveModule(1, new SwerveModuleConstants(
      SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_ENCODER, 
      SwerveConstants.FRONT_RIGHT_STEER_OFFSET)),

      new SwerveModule(2, new SwerveModuleConstants(
      SwerveConstants.BACK_LEFT_DRIVE_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_ENCODER, 
      SwerveConstants.BACK_LEFT_STEER_OFFSET)),

      new SwerveModule(3, new SwerveModuleConstants(
      SwerveConstants.BACK_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_ENCODER, 
      SwerveConstants.BACK_RIGHT_STEER_OFFSET))
    };

    signals = new BaseStatusSignal[16];
    for(int i = 0; i<4; i++) {
      BaseStatusSignal[] tempSignals = swerveModules[i].getSignals();
      signals[i*4+0] = tempSignals[0];
      signals[i*4+1] = tempSignals[1];
      signals[i*4+2] = tempSignals[2];
      signals[i*4+3] = tempSignals[3];
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("0 Speed", swerveModules[0].getMotorSpeed());
    SmartDashboard.putNumber("1 Speed", swerveModules[1].getMotorSpeed());
    SmartDashboard.putNumber("2 Speed", swerveModules[2].getMotorSpeed());
    SmartDashboard.putNumber("3 Speed", swerveModules[3].getMotorSpeed());
    SmartDashboard.putNumber("FL", swerveModules[0].getEncoderAngle().getRotations());
    SmartDashboard.putNumber("FR", swerveModules[1].getEncoderAngle().getRotations());
    SmartDashboard.putNumber("BL", swerveModules[2].getEncoderAngle().getRotations());
    SmartDashboard.putNumber("BR", swerveModules[3].getEncoderAngle().getRotations());
    SmartDashboard.putNumber("FLM", swerveModules[0].getMotorAngle().getRotations());
    SmartDashboard.putNumber("FRM", swerveModules[1].getMotorAngle().getRotations());
    SmartDashboard.putNumber("BLM", swerveModules[2].getMotorAngle().getRotations());
    SmartDashboard.putNumber("BRM", swerveModules[3].getMotorAngle().getRotations());
    
  }

  /**
   * Main controlling method for driving swerve based on desired speed of drivetrian
   * @param chassisSpeeds Desired speed of drivetrain
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  /**
   * Main controlling method for driving swerve based on desired speed of drivetrian
   * @param chassisSpeeds Desired speed of drivetrain
   */
  public void betterDrive(ChassisSpeeds chassisSpeeds) {
    BetterSwerveModuleState[] states = SwerveConstants.BETTER_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    betterSetModuleStates(states);
  }

  /**
   * Set the desired state of all the modules
   * @param states Desired module states
   */
  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    swerveModules[0].setDesiredState(states[0]);
    swerveModules[1].setDesiredState(states[1]);
    swerveModules[2].setDesiredState(states[2]); 
    swerveModules[3].setDesiredState(states[3]); 
  }

  /**
   * Set the desired state of all the modules
   * @param states Desired module states
   */
  public void betterSetModuleStates(BetterSwerveModuleState[] states){
    BetterSwerveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    swerveModules[0].setDesiredState(states[0]);
    swerveModules[1].setDesiredState(states[1]);
    swerveModules[2].setDesiredState(states[2]); 
    swerveModules[3].setDesiredState(states[3]); 
  }

  /**
   * Stops all movement for swerve modules
   */
  public void stop(){
    swerveModules[0].stop();
    swerveModules[1].stop();
    swerveModules[2].stop();
    swerveModules[3].stop();
  }

  /**
   * Rotates modules in an X shape which makes it hard to push
   */
  public void lock(){
    swerveModules[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
    swerveModules[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    swerveModules[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    swerveModules[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
  }


  public void reset(){

    swerveModules[0].setDesiredState(new SwerveModuleState(0.0, new Rotation2d().minus(swerveModules[0].getEncoderAngle())));
    swerveModules[1].setDesiredState(new SwerveModuleState(0.0, new Rotation2d().minus(swerveModules[1].getEncoderAngle())));
    swerveModules[2].setDesiredState(new SwerveModuleState(0.0, new Rotation2d().minus(swerveModules[2].getEncoderAngle())));
    swerveModules[3].setDesiredState(new SwerveModuleState(0.0, new Rotation2d().minus(swerveModules[3].getEncoderAngle())));

  }

  /**
   * Get the position of all the modules including distance and angle of each
   * @return Position of all the modules as an array
   */
  public SwerveModulePosition[] getPositions(boolean refresh) {
    return new SwerveModulePosition[] {
      swerveModules[0].getPosition(refresh),
      swerveModules[1].getPosition(refresh),
      swerveModules[2].getPosition(refresh),
      swerveModules[3].getPosition(refresh)
    };
  }

  public BaseStatusSignal[] getSignals() {
    return signals;
  }

  /**
   * Get the state of all the modules including velocity and angle of each
   * @return The state of all the modules as an array
   */
  public SwerveModuleState[] getStates(boolean refresh) {
    return new SwerveModuleState[] {
      swerveModules[0].getState(refresh),
      swerveModules[1].getState(refresh),
      swerveModules[2].getState(refresh),
      swerveModules[3].getState(refresh)
    };
  }

  /**
   * @return the current speed of the robot in whatever direction it is traveling
   */
  public double getCurrentChassisSpeeds(boolean refresh) {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates(refresh));
    double linearVelocity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVelocity;
  }

  /**
   * @param currentPose of the robot from the pose estimator
   * @param refresh 
   * @return the current direction the robot is traveling in
   */
  public Rotation2d getCurrentChassisHeading(Pose2d currentPose, boolean refresh) {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates(refresh));
    Rotation2d robotHeading = new Rotation2d(Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    Rotation2d currentHeading = robotHeading.plus(currentPose.getRotation());
    return currentHeading;
  }
}
