// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Lift extends SubsystemBase {

    private CANSparkMax lift;
    private RelativeEncoder m_Encoder;

    public Lift() {
        lift = new CANSparkMax(Constants.LiftConstants.LiftSparkmaxDeviceID, MotorType.kBrushless);
        lift.restoreFactoryDefaults();
        lift.setIdleMode(IdleMode.kBrake);
        lift.setSmartCurrentLimit(80);
        lift.setInverted(true);

        m_Encoder = lift.getEncoder();
        m_Encoder.setPosition(0);
        // lift.setSoftLimit(SoftLimitDirection.kForward, Constants.LiftConstants.Extended);
        // lift.setSoftLimit(SoftLimitDirection.kReverse, Constants.LiftConstants.Home);
        // lift.enableSoftLimit(SoftLimitDirection.kForward, true);
        // lift.enableSoftLimit(SoftLimitDirection.kReverse, true);
        lift.burnFlash();

    }

    public void periodic() {
        SmartDashboard.putNumber("Lift_Encoder", m_Encoder.getPosition());
    }

    public void up() {
        lift.set(Constants.LiftConstants.up_speed);
    }

    public void down() {
        lift.set(Constants.LiftConstants.down_speed);
    }

    public void stop() {
        lift.stopMotor();
    }

}
