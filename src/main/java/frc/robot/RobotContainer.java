// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveWithJoysticks;
import frc.robot.Subsystems.FiringHead;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pigeon2Subsystem;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.PoseEstimator;
import frc.robot.Subsystems.SwerveSubsystem;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

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

  public RobotContainer() {

    swerveSubsystem.reset();
    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks(
        swerveSubsystem,
        poseEstimator,
        () -> driverController.getLeftY(),
        () -> driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> GlobalVariables.fieldRelative,
        () -> GlobalVariables.maxSpeed));
    configureBindings();
  }

  private void configureBindings() {
    driverController.back().onTrue(new InstantCommand(() -> poseEstimator.setPose(new Pose2d()), poseEstimator));
    driverController.x().onTrue(new InstantCommand(() -> GlobalVariables.fieldRelative = !GlobalVariables.fieldRelative));
    driverController.b().onTrue(new InstantCommand(() -> swerveSubsystem.lock(), swerveSubsystem));
    driverController.rightBumper().whileTrue(new InstantCommand(() -> firingHead.Fire(), firingHead));
    driverController.rightBumper().onFalse(new InstantCommand(() -> firingHead.Stop(), firingHead));


    operatorController.rightBumper().onTrue(new InstantCommand(() -> intake.on(), intake));
    operatorController.leftBumper().onTrue(new InstantCommand(() -> intake.off(), intake));




  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
  }
}
