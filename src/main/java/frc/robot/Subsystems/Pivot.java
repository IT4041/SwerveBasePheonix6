// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Pivot extends SubsystemBase {
 

  private CANSparkMax mainMotor;
  private SparkPIDController m_pidController;
  private SparkAbsoluteEncoder m_Encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  
  public Pivot() {
    mainMotor = new CANSparkMax(Constants.PivotConstants.SparkmaxDeviceID,MotorType.kBrushless);

    mainMotor.restoreFactoryDefaults();
    
    //m_pidController = mainMotor.getPIDController();
    m_Encoder = mainMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // m_Encoder.setPositionConversionFactor(360);
    // m_Encoder.setVelocityConversionFactor(1);
    // m_Encoder.setInverted(false);
    // m_Encoder.setZeroOffset(Constants.PivotConstants.PivotPIDConstants.offset);

    // kP = Constants.PivotConstants.PivotPIDConstants.kP;
    // kI = Constants.PivotConstants.PivotPIDConstants.kI;
    // kD = Constants.PivotConstants.PivotPIDConstants.kD;
    // kIz = Constants.PivotConstants.PivotPIDConstants.kIz;
    // kFF = Constants.PivotConstants.PivotPIDConstants.kFF;
    // kMaxOutput = Constants.PivotConstants.PivotPIDConstants.kMaxOutput;
    // kMinOutput = Constants.PivotConstants.PivotPIDConstants.kMinOutput;
    
    // m_pidController.setP(kP);
    // m_pidController.setI(kI);
    // m_pidController.setD(kD);
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    mainMotor.setIdleMode(IdleMode.kBrake);
    mainMotor.setSmartCurrentLimit(80);
    mainMotor.setClosedLoopRampRate(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("getEncoderRate", m_Encoder.getVelocity());
    
  }
  public void setPosition(double position){
    m_pidController.setReference(position, CANSparkMax.ControlType.kPosition, 0, 0, ArbFFUnits.kPercentOut);
  }
  public void Dump() {
    this.setPosition(Constants.PivotConstants.PivotPIDConstants.DumpPoint);
  }

  public void Starting() {
   this.setPosition(Constants.PivotConstants.PivotPIDConstants.StartingPoint);
  }

  public void Shooting() {
   this.setPosition(Constants.PivotConstants.PivotPIDConstants.ShootingPoint);
  }
  public void TestingOn() {
    mainMotor.set(0.4);
  }
  public void TestingOff() {
    mainMotor.stopMotor();
  }
}
