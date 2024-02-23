// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveWithJoysticks;
import frc.robot.Commands.Autonomous.AutoSequences;
import frc.robot.Subsystems.FiringHead;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Pigeon2Subsystem;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.PoseEstimator;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.MasterController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

  private SendableChooser<Command> trajChooser;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();
  private final PoseEstimator poseEstimator = new PoseEstimator(swerveSubsystem, pigeon2Subsystem);

  public static final CommandXboxController driverController = new CommandXboxController(
      Constants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(
      Constants.kOperatorControllerPort);

  private final Pivot pivot = new Pivot();
  private final Intake intake = new Intake();
  private final FiringHead firingHead = new FiringHead();
  private final Lift lift = new Lift();
  private final LED led = new LED();
  private final MasterController masterController = new MasterController(pivot, intake, firingHead, led);

  public RobotContainer() {

    swerveSubsystem.reset();
    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks(
        swerveSubsystem,
        poseEstimator,
        () -> driverController.getLeftY(),
        () -> driverController.getLeftX(),
        () -> driverController.getRightX(),
        () -> Constants.fieldRelative,
        () -> Constants.DRIVE_SPEED));

    // ParallelCommandGroup ShootPrep = new ParallelCommandGroup(
    //   new InstantCommand(() ->  pivot.ShootingShortRange(), pivot),
    //   new InstantCommand(() ->  firingHead.shooterSetSpeed(masterController.getFiringSpeed()), firingHead));

    // firingHead.setDefaultCommand(ShootPrep);

    configureBindings();

    AutoSequences autoSeq = new AutoSequences(pivot, intake, firingHead, masterController);

    NamedCommands.registerCommand("near_shooting", new InstantCommand(() -> firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FiringSpeed),firingHead));
    NamedCommands.registerCommand("far_shooting", new InstantCommand(() -> firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FarFiringSpeed),firingHead));
    NamedCommands.registerCommand("dump_shooting", new InstantCommand(() -> firingHead.shooterSetSpeed(Constants.FiringHeadConstants.DumpSpeed),firingHead));

    NamedCommands.registerCommand("pivot_dump", new InstantCommand(() -> pivot.goToPosition(Constants.PivotConstants.PivotPostions.DumpPoint),pivot));
    NamedCommands.registerCommand("pivot_mid", new InstantCommand(() -> pivot.goToPosition(Constants.PivotConstants.PivotPostions.ShootingPointMidRange),pivot));
    NamedCommands.registerCommand("pivot_short", new InstantCommand(() -> pivot.goToPosition(Constants.PivotConstants.PivotPostions.ShootingPointShortRange),pivot));
    NamedCommands.registerCommand("pivot_start", new InstantCommand(() -> pivot.goToPosition(Constants.PivotConstants.PivotPostions.StartingPoint),pivot));

    NamedCommands.registerCommand("starting_sequence", autoSeq.AutoStartingSequence());
    NamedCommands.registerCommand("run_conveyors", autoSeq.AutoConveyorSequence());
    NamedCommands.registerCommand("stop_conveyors", autoSeq.AutoStopSequence());

    NamedCommands.registerCommand("fire_dump", autoSeq.AutoShootingSequence(Constants.FiringHeadConstants.DumpSpeed,Constants.PivotConstants.PivotPostions.DumpPoint));
    NamedCommands.registerCommand("fire_near", autoSeq.AutoShootingSequence(Constants.FiringHeadConstants.FiringSpeed,Constants.PivotConstants.PivotPostions.ShootingPointShortRange));
    NamedCommands.registerCommand("fire_far", autoSeq.AutoShootingSequence(Constants.FiringHeadConstants.FarFiringSpeed,Constants.PivotConstants.PivotPostions.ShootingPointMidRange));

    trajChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", trajChooser);
  }

  private void configureBindings() {

    SequentialCommandGroup home = new SequentialCommandGroup(
        new InstantCommand(() -> firingHead.shooterSetSpeed(0), firingHead), // shooter off
        new InstantCommand(() -> intake.setIntakeSpeed(0), intake), // intake off
        new InstantCommand(() -> firingHead.setTransportMotorSpeed(0), firingHead), // transport motor off
        new InstantCommand(() -> intake.setConveyorSpeed(0), intake), // conveyr off
        new InstantCommand(() -> pivot.Starting(), pivot) // pivot starting position
    );

    driverController.rightTrigger().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> firingHead.shooterSetSpeed(masterController.getFiringSpeed()), firingHead), // shooter off
            new WaitCommand(.1),
            new InstantCommand(() -> firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed), firingHead), // transport motor off
            new WaitCommand(1), // conveyr off
            new InstantCommand(() -> firingHead.MasterStop(), firingHead),
            new InstantCommand(() -> pivot.Starting(), pivot)));

    driverController.leftTrigger().onTrue(new RunCommand(() -> firingHead.source(), firingHead)
        .until(() -> (firingHead.SideSensorTriggered()))
        .andThen(new InstantCommand(() -> firingHead.MasterStop(), firingHead)));        

    driverController.start().onTrue(home);

    //********************* operator control **************************/
    operatorController.x().whileTrue(new InstantCommand(() -> lift.up(), lift));
    operatorController.x().onFalse(new InstantCommand(() -> lift.stop(), lift));

    operatorController.b().whileTrue(new InstantCommand(() -> lift.down(), lift));
    operatorController.b().onFalse(new InstantCommand(() -> lift.stop(), lift));

    operatorController.start().onTrue(home); // pivot starting position

    operatorController.y().onTrue(new InstantCommand(() -> pivot.up(), pivot));
    operatorController.a().onTrue(new InstantCommand(() -> pivot.down(), pivot));

    operatorController.rightTrigger().onTrue(new RunCommand(() -> masterController.runConveyors(), masterController)
        .until(() -> (firingHead.EitherSensorTriggered() && pivot.InStartingPosition()) || (intake.EitherSensorTriggered() && !pivot.InStartingPosition()))
        .andThen(new InstantCommand(() -> masterController.stopConveyors(), masterController)));
  }

  public Command getAutonomousCommand() {
    return trajChooser.getSelected();
  }
}
