// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveWithJoysticks;
import frc.robot.Commands.Autonomous.AutoSequences;
import frc.robot.Commands.Autonomous.WeekZeroAuto;
import frc.robot.Subsystems.FiringHead;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Pigeon2Subsystem;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.PoseEstimator;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.MasterController;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;


public class RobotContainer {

  private AutoSequences autoSequences;
  private SendableChooser<Command> trajChooser;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();
  private final PoseEstimator poseEstimator = new PoseEstimator(swerveSubsystem, pigeon2Subsystem);

  public static final CommandXboxController driverController = new CommandXboxController(Constants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(Constants.kOperatorControllerPort);

  private final Pivot pivot = new Pivot();
  private final Intake intake = new Intake();
  private final FiringHead firingHead = new FiringHead();
  private final Lift lift = new Lift();
  private final MasterController masterController = new MasterController(pivot, intake, firingHead);

  public RobotContainer() {

    autoSequences = new AutoSequences();

    swerveSubsystem.reset();
    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks(
        swerveSubsystem,
        poseEstimator,
        () -> driverController.getLeftY(),
        () -> driverController.getLeftX(),
        () -> driverController.getRightX(),
        () -> Constants.fieldRelative,
        () -> Constants.DRIVE_SPEED));
    configureBindings();
    setupTrajectoryDashboardChooser();
  }

  private void configureBindings() {
    
    SequentialCommandGroup home = new SequentialCommandGroup(
    new InstantCommand(() -> firingHead.shooterSetSpeed(0), firingHead), //shooter off
    new InstantCommand(() -> intake.setIntakeSpeed(0), intake), //intake off
    new InstantCommand(() -> firingHead.setTransportMotorSpeed(0), firingHead), //transport motor off
    new InstantCommand(() -> intake.setConveyrSpeed(0), intake), //conveyr off
    new InstantCommand(() -> pivot.Starting(), pivot) //pivot starting position
    );

    driverController.start().onTrue(home);
    
    operatorController.y().onTrue(new InstantCommand(() -> pivot.up(), pivot));
    operatorController.a().onTrue(new InstantCommand(() -> pivot.down(), pivot));
    
    operatorController.b().onTrue(new RunCommand(() -> masterController.runConveyors(),masterController)
    .until(() -> (firingHead.EitherSensorTriggered() && pivot.InStartingPosition()) || (intake.EitherSensorTriggered() && !pivot.InStartingPosition()))
    .andThen(new InstantCommand(() -> masterController.stopConveyors(),masterController)));

    driverController.rightTrigger().onTrue(
      new SequentialCommandGroup(
      new InstantCommand(() -> firingHead.shooterSetSpeed(masterController.getFiringSpeed()), firingHead), //shooter off
      new WaitCommand(1),
      new InstantCommand(() -> firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed), firingHead), //transport motor off
      new WaitCommand(3 ), //conveyr off
      new InstantCommand(() -> firingHead.MasterStop(), firingHead))
      );

    operatorController.x().whileTrue(new InstantCommand(() -> lift.up(), lift));
    operatorController.b().whileTrue(new InstantCommand(() -> lift.down(), lift));

    operatorController.start().onTrue(home); //pivot starting position
  }
  
  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("New Auto");
    return new WeekZeroAuto(pivot, intake, firingHead, masterController, swerveSubsystem);
    //return trajChooser.getSelected();
  }

  public void setupTrajectoryDashboardChooser(){
    trajChooser = AutoBuilder.buildAutoChooser();
  }
}
