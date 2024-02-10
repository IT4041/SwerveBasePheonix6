// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FiringHead extends SubsystemBase {
  
  private CANSparkMax fireMotor;
  private CANSparkMax followMotor;

  private CANSparkMax transportMotor;

  


  
  /** Creates a new FiringHead. */
  public FiringHead() {
    fireMotor = new CANSparkMax(Constants.FiringHeadConstants.UpperSparkmaxDeviceID,MotorType.kBrushless);
    followMotor = new CANSparkMax(Constants.FiringHeadConstants.LowerSparkmaxDeviceID,MotorType.kBrushless);
    fireMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();
    
    transportMotor = new CANSparkMax(Constants.FiringHeadConstants.UpperTransportSparkmaxDeviceID,MotorType.kBrushless);
    transportMotor.restoreFactoryDefaults();

    fireMotor.setIdleMode(IdleMode.kBrake);
    fireMotor.setSmartCurrentLimit(80);
    fireMotor.setClosedLoopRampRate(1);

    followMotor.setIdleMode(IdleMode.kBrake);
    followMotor.setSmartCurrentLimit(80);
    followMotor.setClosedLoopRampRate(1);

    transportMotor.setIdleMode(IdleMode.kBrake);
    transportMotor.setSmartCurrentLimit(80);
    transportMotor.setClosedLoopRampRate(1);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void fire() {
    
  }
}
