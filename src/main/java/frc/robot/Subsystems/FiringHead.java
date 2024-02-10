// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class FiringHead extends SubsystemBase {
  
  private CANSparkMax fireMotor;
  private CANSparkMax followMotor;

  private CANSparkMax transportMotor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_Encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, cRPM;
  


  
  /** Creates a new FiringHead. */
  public FiringHead() {
    fireMotor = new CANSparkMax(Constants.FiringHeadConstants.UpperSparkmaxDeviceID,MotorType.kBrushless);
    followMotor = new CANSparkMax(Constants.FiringHeadConstants.LowerSparkmaxDeviceID,MotorType.kBrushless);
    transportMotor = new CANSparkMax(Constants.FiringHeadConstants.UpperTransportSparkmaxDeviceID,MotorType.kBrushless);
    
    fireMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();
    transportMotor.restoreFactoryDefaults();

    //m_pidController = fireMotor.getPIDController();
    m_Encoder = fireMotor.getEncoder();

    // kP = Constants.FiringHeadConstants.FiringHeadPIDConstants.kP;
    // kI = Constants.FiringHeadConstants.FiringHeadPIDConstants.kI;
    // kD = Constants.FiringHeadConstants.FiringHeadPIDConstants.kD;
    // kIz = Constants.FiringHeadConstants.FiringHeadPIDConstants.kIz;
    // kFF = Constants.FiringHeadConstants.FiringHeadPIDConstants.kFF;
    // kMaxOutput = Constants.FiringHeadConstants.FiringHeadPIDConstants.kMaxOutput;
    // kMinOutput = Constants.FiringHeadConstants.FiringHeadPIDConstants.kMinOutput;
    
    // m_pidController.setP(kP);
    // m_pidController.setI(kI);
    // m_pidController.setD(kD);
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    fireMotor.setIdleMode(IdleMode.kBrake);
    fireMotor.setSmartCurrentLimit(80);
    fireMotor.setClosedLoopRampRate(1);

    followMotor.setIdleMode(IdleMode.kBrake);
    followMotor.setSmartCurrentLimit(80);
    followMotor.setClosedLoopRampRate(1);

    transportMotor.setIdleMode(IdleMode.kBrake);
    transportMotor.setSmartCurrentLimit(80);
    transportMotor.setClosedLoopRampRate(1);

    followMotor.follow(fireMotor, true);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // if (m_Encoder.getVelocity() >= CalculateRPM(0)) {
      
    // }
    //SmartDashboard.putNumber("getEncoderRate", m_Encoder.getVelocity());

  }
  private double CalculateRPM(double distance) {
    cRPM = 0;
    
    return cRPM;
  }
  public void Feed() {
    transportMotor.set(0.4);
    fireMotor.set(0.4);
  }
  public void SpeedUp() {
    //m_pidController.setReference(Constants.FiringHeadConstants.FiringHeadPIDConstants.FireVelocity, CANSparkMax.ControlType.kVelocity);
   
    
  }
 
  public void Fire() {
    m_pidController.setReference(Constants.FiringHeadConstants.FiringHeadPIDConstants.FireVelocity, CANSparkMax.ControlType.kVelocity);

  
  }
  public void Stop() {
    transportMotor.stopMotor();
    fireMotor.stopMotor();
  }
}
