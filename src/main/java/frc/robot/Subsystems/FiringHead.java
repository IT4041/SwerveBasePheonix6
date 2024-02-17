// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FiringHead extends SubsystemBase {

  private CANSparkMax fireMotor;
  private CANSparkMax followMotor;
 


  enum Stages {
    Idle,
    Triggered,
    Paused,
    Fired
  }

  Stages stage = Stages.Idle;

  private final TimeOfFlight centerSensor = new TimeOfFlight(Constants.FiringHeadConstants.TimeOfFlightASensorID);
  private final TimeOfFlight sideSensor = new TimeOfFlight(Constants.FiringHeadConstants.TimeOfFlightBSensorID);

  private CANSparkMax transportMotor;
  private RelativeEncoder m_Encoder;

  /** Creates a new FiringHead. */
  public FiringHead() {
    fireMotor = new CANSparkMax(Constants.FiringHeadConstants.UpperSparkmaxDeviceID, MotorType.kBrushless);
    followMotor = new CANSparkMax(Constants.FiringHeadConstants.LowerSparkmaxDeviceID, MotorType.kBrushless);
    transportMotor = new CANSparkMax(Constants.FiringHeadConstants.UpperTransportSparkmaxDeviceID,
        MotorType.kBrushless);

    fireMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();
    transportMotor.restoreFactoryDefaults();

    m_Encoder = fireMotor.getEncoder();

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

    centerSensor.setRangingMode(RangingMode.Short, 1);
    sideSensor.setRangingMode(RangingMode.Short, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putString("Firing Stage", stage.toString());
    SmartDashboard.putNumber("firing head velocity", m_Encoder.getVelocity());

    SmartDashboard.putBoolean("SH Center triggered?", this.CenterSensorTriggered());
    SmartDashboard.putBoolean("SH Side triggered?", this.SideSensorTriggered());

    SmartDashboard.putNumber("SH Center distance", centerSensor.getRange());
    SmartDashboard.putNumber("SH Side distance", sideSensor.getRange());

    SmartDashboard.putBoolean("conveyr on", transportMotor.get()>0);
    SmartDashboard.putBoolean("shooter head is on", fireMotor.get()>0);

  }

  public void Feed() {
  
    transportMotor.set(Constants.FiringHeadConstants.TransportMotorSpeed);
    fireMotor.set(Constants.FiringHeadConstants.FiringSpeed);
  }

  public void StopTransport() {
    transportMotor.stopMotor();
    stage = Stages.Paused;
  }

  public boolean CenterSensorTriggered() {
    return centerSensor.getRange() <= Constants.FiringHeadConstants.NoIntakeThresholdA;
  }

  public boolean SideSensorTriggered() {
    return sideSensor.getRange() <= Constants.FiringHeadConstants.NoIntakeThresholdB;
  }

  public boolean EitherSensorTriggered(){
    return this.SideSensorTriggered() || this.CenterSensorTriggered();
  }

  public void MasterStop() {
    transportMotor.stopMotor();
    fireMotor.stopMotor();
    stage = Stages.Idle;
  }

  public void shooterSetSpeed(double speed){
      fireMotor.set(speed);
  }
  public void setTransportMotorSpeed(double speed){
    transportMotor.set(speed);
  }

  public TimeOfFlight getSensorA(){
    return centerSensor;
  }


 
}
