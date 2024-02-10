// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private static Pivot m_inst = null;

  private CANSparkMax mainMotor;
  
  
  
  public Pivot() {
    mainMotor = new CANSparkMax(0,MotorType.kBrushless);

    mainMotor.restoreFactoryDefaults();
    
    mainMotor.setIdleMode(IdleMode.kBrake);
    mainMotor.setSmartCurrentLimit(80);
    mainMotor.setClosedLoopRampRate(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
