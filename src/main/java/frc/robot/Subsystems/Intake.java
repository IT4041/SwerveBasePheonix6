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

private int Stage;

private final TimeOfFlight rangeSensorIntake = new TimeOfFlight(Constants.IntakeConstants.TimeOfFlightSensorId);

  public Intake() {

    //lower intake
    intake = new CANSparkMax(Constants.IntakeConstants.LowerIntakeSparkmaxDeviceID,MotorType.kBrushless);
    intake.restoreFactoryDefaults();
    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(80);
    intake.setClosedLoopRampRate(1);
    Stage = 0;

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



    rangeSensorIntake.setRangingMode(RangingMode.Long, 1);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putBoolean("rangeSensorIntake triggered?",this.IntakeTriggered());

    if(Stage == 1 && this.IntakeTriggered()){
      //this.off();
      
      intake.stopMotor();
      
      Stage = 2; 

    }
  }

  public boolean IntakeTriggered(){
    return rangeSensorIntake.getRange() <= Constants.IntakeConstants.ballTreshholdIntake;
  }

  public void on(){
    
    conveyrLow.set(Constants.IntakeConstants.ConveyrMotorSpeed);
    conveyrUp.set(-Constants.IntakeConstants.ConveyrMotorSpeed);
   

    if(Stage == 0){
      
      intake.set(Constants.IntakeConstants.IntakeMotorSpeed);
      
      Stage = 1;

    } else {
      
      if(Stage == 2){
        
        Stage = 3;

      }
    }
    
  
  }

  public void off(){
    intake.stopMotor();
    conveyrLow.stopMotor();
    conveyrUp.stopMotor();
    
    Stage = 0;

  }
}
