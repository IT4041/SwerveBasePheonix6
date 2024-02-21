// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.FiringHead;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.MasterController;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

/** Add your docs here. */
public class AutoSequences {
  private final Intake m_intake;
  private final Pivot m_pivot;
  private final FiringHead m_firingHead;
  private final MasterController m_masterController;

  
  public AutoSequences(Pivot in_pivot, Intake in_intake, FiringHead in_firingHead, MasterController in_masterController) {
    m_intake = in_intake;
    m_pivot = in_pivot;
    m_firingHead = in_firingHead;
    m_masterController = in_masterController;
}
    public SequentialCommandGroup AutoStartingSequence(){

        SequentialCommandGroup group = new SequentialCommandGroup(  
        new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FiringSpeed), m_firingHead),
        new InstantCommand(() -> m_pivot.goToPosition(Constants.PivotConstants.PivotPostions.ShootingPointShortRange), m_pivot), //27
        new WaitCommand(0.5),
        new InstantCommand(() -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed), m_firingHead),
        new WaitCommand(1),
        new InstantCommand(() -> m_firingHead.shooterSetSpeed(0), m_firingHead),
        new InstantCommand(() -> m_pivot.goToPosition(Constants.PivotConstants.PivotPostions.StartingPoint), m_pivot),
        new InstantCommand(() -> m_intake.setIntakeSpeed(Constants.IntakeConstants.IntakeMotorSpeed), m_intake),
        new InstantCommand(() -> m_intake.setConveyrSpeed(Constants.IntakeConstants.ConveyrMotorSpeed), m_intake));

        return group;
    }

    public SequentialCommandGroup AutoConveyrSequence(){

        SequentialCommandGroup command = new RunCommand(() -> m_masterController.runConveyors(),m_masterController)
        .until(() -> m_firingHead.EitherSensorTriggered())
        .andThen(new InstantCommand(() -> m_masterController.stopConveyors(),m_masterController));

        return command;
    }
    
}
