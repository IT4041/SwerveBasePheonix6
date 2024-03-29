// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MasterController extends SubsystemBase {

  private final Intake m_intake;
  private final Pivot m_pivot;
  private final FiringHead m_firingHead;
  private final LED m_led;

  public MasterController(Pivot in_pivot, Intake in_intake, FiringHead in_firingHead, LED in_led) {
    m_intake = in_intake;
    m_pivot = in_pivot;
    m_firingHead = in_firingHead;
    m_led = in_led;
  }

  @Override
  public void periodic() {

    // change leds if we have a note
    if (this.anySensorTriggered()) {
      m_led.capturedNote();
    } else {
      m_led.noNote();
    }

    //if we have a note in firing head move to short shooting position
    // and turn on shooting wheels
    if(m_firingHead.EitherSensorTriggered()){
      //TODO: this needs to be overridden when the operator moves the pivot
      if(m_pivot.InStartingPosition()){
        m_pivot.GoToShootingShortRange();
      }
      m_firingHead.shooterSetSpeed(this.getFiringSpeed());
    }
  }

  public void intake_on() {
    m_intake.on();
  }

  public void intake_off() {
    m_intake.off();
  }

  public void firingHead_feed() {
    m_firingHead.Feed();
  }

  public void firingHead_MasterStop() {
    m_firingHead.MasterStop();
  }

  public void pivot_shooting() {
    m_pivot.GoToShootingShortRange();
  }

  public void pivot_starting() {
    m_pivot.GoToStarting();
  }

  public void pivot_dump() {
    m_pivot.GoToDump();
  }

  public void runConveyors() {
    m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed);
    m_intake.setConveyorSpeed(Constants.IntakeConstants.ConveyrMotorSpeed);
    m_intake.setIntakeSpeed(Constants.IntakeConstants.IntakeMotorSpeed);
  }

  public void stopConveyors() {
    m_firingHead.setTransportMotorSpeed(0);
    m_intake.setConveyorSpeed(0);
    m_intake.setIntakeSpeed(0);
  }

  public double getFiringSpeed() {
    double retSpeed;
    switch (m_pivot.ReturnPositionIndex()) {
      case 3:
        retSpeed = Constants.FiringHeadConstants.DumpSpeed;
        break;
      case 2:
        retSpeed = Constants.FiringHeadConstants.FarFiringSpeed;
        break;
      case 1:
        retSpeed = Constants.FiringHeadConstants.FiringSpeed;
        break;
      case 0:
        retSpeed = Constants.FiringHeadConstants.FiringSpeed;
        break;
      default:retSpeed = Constants.FiringHeadConstants.FiringSpeed;
        break;
    }
    return retSpeed;
  }

  private boolean anySensorTriggered() {
    return m_firingHead.EitherSensorTriggered() || m_intake.EitherSensorTriggered();
  }
}
