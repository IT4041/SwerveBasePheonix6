// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WeekZeroAuto extends SequentialCommandGroup {
  private final Intake m_intake;
  private final Pivot m_pivot;
  private final FiringHead m_firingHead;
  private final MasterController m_masterController;
  /** Creates a new WeekZeroAuto. */
  public WeekZeroAuto(Pivot in_pivot, Intake in_intake, FiringHead in_firingHead, MasterController in_masterController) {
    m_intake = in_intake;
    m_pivot = in_pivot;
    m_firingHead = in_firingHead;
    m_masterController = in_masterController;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FiringSpeed), m_firingHead));
    addCommands(new InstantCommand(() -> m_pivot.goToPosition(Constants.PivotConstants.PivotPostions.ShootingPointShortRange), m_pivot)); //27
    addCommands(new WaitCommand(2));
    addCommands(new InstantCommand(() -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed), m_firingHead));
    addCommands(new WaitCommand(1));
    addCommands(new InstantCommand(() -> m_firingHead.shooterSetSpeed(0), m_firingHead));
     //addCommand(new InstantCommand(()->move 4 feet))
     //add automatic turnOff intakes command
    addCommands(new WaitCommand(3));
    addCommands(new InstantCommand(() -> m_firingHead.setTransportMotorSpeed(0), m_firingHead));

   //************************************/
    addCommands(new InstantCommand(() -> m_pivot.goToPosition(Constants.PivotConstants.PivotPostions.StartingPoint), m_pivot));
    addCommands(new InstantCommand(() -> m_intake.setIntakeSpeed(Constants.IntakeConstants.IntakeMotorSpeed), m_intake));
    addCommands(new InstantCommand(() -> m_intake.setConveyrSpeed(Constants.IntakeConstants.ConveyrMotorSpeed), m_intake));
    addCommands(new InstantCommand(() -> m_masterController.runIntakeUntilTriggered(m_firingHead.getSensorA(), Constants.FiringHeadConstants.NoIntakeThresholdA), m_masterController));
    addCommands(new InstantCommand(() -> m_firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FiringSpeed), m_firingHead));
    addCommands(new InstantCommand(() -> m_pivot.goToPosition(Constants.PivotConstants.PivotPostions.ShootingPointMidRange), m_pivot)); //37
    addCommands(new WaitCommand(2));
    addCommands(new InstantCommand(() -> m_firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed), m_firingHead)); 
    addCommands(new WaitCommand(2));
    //add new parallel command
    addCommands(new ParallelCommandGroup(
      new InstantCommand(() -> m_firingHead.setTransportMotorSpeed(0), m_firingHead), //conveyrs
      new InstantCommand(() -> m_firingHead.shooterSetSpeed(0), m_firingHead), //shooter
      new InstantCommand(() -> m_intake.setIntakeSpeed(0), m_intake) //intake
    ));
  }
}
