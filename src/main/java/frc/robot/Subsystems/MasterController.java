// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MasterController extends SubsystemBase {

  private final Intake m_intake;
  private final Pivot m_pivot;
  private final FiringHead m_firingHead;
  private boolean intake_on = false;

  public MasterController(Pivot in_pivot, Intake in_intake, FiringHead in_firingHead) {
    m_intake = in_intake;
    m_pivot = in_pivot;
    m_firingHead = in_firingHead;
  }

  // intake

  public void intake_on() {
    m_intake.on();
    intake_on = true;
  }

  public void intake_off() {
    m_intake.off();
    intake_on = false;
  }

  // firing head

  public void firingHead_feed() {
    m_firingHead.Feed();
  }

  public void firingHead_MasterStop() {
    m_firingHead.MasterStop();
  }

  // pivot

  public void pivot_shooting() {
    m_pivot.ShootingShortRange();
  }

  public void pivot_starting() {
    m_pivot.Starting();
  }

  public void pivot_dump() {
    m_pivot.Dump();
  }

  @Override
  public void periodic() {
    if (intake_on) {
      if (m_pivot.InStartingPosition()) {
        m_firingHead.Feed();
      }
    }
  }
}
