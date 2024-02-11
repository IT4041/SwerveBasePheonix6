// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
private CANSparkMax intake;
private CANSparkMax conveyrUp;
private CANSparkMax conveyrLow;

private final TimeOfFlight rangeSensorIntake = new TimeOfFlight(Constants.IntakeConstants.TimeOfFlightSensorId);
public boolean isOn;

  public Intake() {

    //lower intake
    intake = new CANSparkMax(Constants.IntakeConstants.LowerIntakeSparkmaxDeviceID,MotorType.kBrushless);
    intake.restoreFactoryDefaults();
    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(80);
    intake.setClosedLoopRampRate(1);
    isOn = false;

    //upper conveyr
    conveyrUp = new CANSparkMax(Constants.IntakeConstants.UpperConvyerSparkmaxDeviceID,MotorType.kBrushless);
    conveyrUp.restoreFactoryDefaults();
    conveyrUp.setIdleMode(IdleMode.kBrake);
    conveyrUp.setSmartCurrentLimit(80);
    conveyrUp.setClosedLoopRampRate(1);

    //lower conveyr
    conveyrLow = new CANSparkMax(Constants.IntakeConstants.LowerConvyerSparkmaxDeviceID,MotorType.kBrushless);
    conveyrLow.restoreFactoryDefaults();
    conveyrLow.setIdleMode(IdleMode.kBrake);
    conveyrLow.setSmartCurrentLimit(80);
    conveyrLow.setClosedLoopRampRate(1);



    rangeSensorIntake.setRangingMode(RangingMode.Short, 1);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("rangeSensorIntake triggered?",this.IntakeTriggered());

    if(isOn && this.IntakeTriggered()){
      //this.off();
    }
  }

  public boolean IntakeTriggered(){
    return rangeSensorIntake.getRange() <= Constants.IntakeConstants.ballTreshholdIntake;
  }

  public void on(){
    intake.set(Constants.IntakeConstants.IntakeMotorSpeed);
    conveyrLow.set(Constants.IntakeConstants.ConveyrMotorSpeed);
    conveyrUp.set(-Constants.IntakeConstants.ConveyrMotorSpeed);
    isOn = true;
  
  }

  public void off(){
    intake.set(0);
    conveyrLow.set(0);
    conveyrUp.set(0);
    isOn = false;
  }
}
