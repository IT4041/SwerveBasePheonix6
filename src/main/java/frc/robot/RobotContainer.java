// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveWithJoysticks;
import frc.robot.Commands.WeekZeroAuto;
import frc.robot.Subsystems.FiringHead;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pigeon2Subsystem;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.PoseEstimator;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.MasterController;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();
  private final PoseEstimator poseEstimator = new PoseEstimator(swerveSubsystem, pigeon2Subsystem);

  public static final CommandXboxController driverController = new CommandXboxController(Constants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(Constants.kOperatorControllerPort);

  private final Pivot pivot = new Pivot();
  private final Intake intake = new Intake();
  private final FiringHead firingHead = new FiringHead();
  private final MasterController masterController = new MasterController(pivot, intake, firingHead);

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
    configureBindings();
  }

  private void configureBindings() {

    driverController.y().onTrue(new InstantCommand(() -> masterController.pivot_shooting(), masterController));
    driverController.b().onTrue(new InstantCommand(() -> masterController.pivot_starting(), masterController));
    driverController.a().onTrue(new InstantCommand(() -> masterController.pivot_dump(), masterController));

    driverController.rightBumper().onTrue(new InstantCommand(() -> masterController.firingHead_feed(), masterController));
    driverController.leftBumper().onTrue(new InstantCommand(() -> masterController.firingHead_MasterStop(), masterController));

    driverController.rightTrigger().onTrue(new InstantCommand(() -> masterController.intake_on(), masterController));
    driverController.leftTrigger().onTrue(new InstantCommand(() -> masterController.intake_off(), masterController));

    operatorController.y().onTrue(new InstantCommand(() -> pivot.up(), pivot));
    operatorController.a().onTrue(new InstantCommand(() -> pivot.down(), pivot));
    //operatorController.b().onTrue(new InstantCommand() -> ... , ...)

    operatorController.rightTrigger().onTrue( new RunCommand(() ->  firingHead.shooterSetSpeed(Constants.FiringHeadConstants.FiringSpeed), firingHead)
    .withTimeout(1)
    .andThen(new InstantCommand(() -> firingHead.setTransportMotorSpeed(Constants.FiringHeadConstants.TransportMotorSpeed), firingHead)));
     
    operatorController.start().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> firingHead.shooterSetSpeed(0), firingHead), //shooter off
      new InstantCommand(() -> intake.setIntakeSpeed(0), intake), //intake off
      new InstantCommand(() -> firingHead.setTransportMotorSpeed(0), firingHead), //transport motor off
      new InstantCommand(() -> intake.setConveyrSpeed(0), intake), //conveyr off
      new InstantCommand(() -> pivot.setPosition(0), pivot) //pivot starting position

    ));
  }
 

  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("New Auto");
    return new WeekZeroAuto(pivot, intake, firingHead, masterController, swerveSubsystem);
  }
}
