// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private static Pivot m_inst = null;

  

  private CANSparkMax mainMotor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_Encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  
  public Pivot() {
    mainMotor = new CANSparkMax(Constants.PivotConstants.SparkmaxDeviceID,MotorType.kBrushless);

    mainMotor.restoreFactoryDefaults();
    
    m_pidController = mainMotor.getPIDController();
    m_Encoder = mainMotor.getEncoder();

    kP = Constants.PivotConstants.PivotPIDConstants.kP;
    kI = Constants.PivotConstants.PivotPIDConstants.kI;
    kD = Constants.PivotConstants.PivotPIDConstants.kD;
    kIz = Constants.PivotConstants.PivotPIDConstants.kIz;
    kFF = Constants.PivotConstants.PivotPIDConstants.kFF;
    kMaxOutput = Constants.PivotConstants.PivotPIDConstants.kMaxOutput;
    kMinOutput = Constants.PivotConstants.PivotPIDConstants.kMinOutput;
    

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

  

    mainMotor.setIdleMode(IdleMode.kBrake);
    mainMotor.setSmartCurrentLimit(80);
    mainMotor.setClosedLoopRampRate(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void Dump() {
    m_pidController.setReference(Constants.PivotConstants.PivotPIDConstants.DumpPoint, CANSparkMax.ControlType.kPosition);
  }

  public void Starting() {
    m_pidController.setReference(Constants.PivotConstants.PivotPIDConstants.StartingPoint, CANSparkMax.ControlType.kPosition);
  }

  public void Shooting() {
    m_pidController.setReference(Constants.PivotConstants.PivotPIDConstants.ShootingPoint, CANSparkMax.ControlType.kPosition);
  }
}
