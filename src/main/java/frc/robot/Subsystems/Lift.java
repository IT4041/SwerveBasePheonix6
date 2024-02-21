// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;

public class Lift extends SubsystemBase {

    private CANSparkMax lift;
    // private RelativeEncoder m_Encoder;

    public Lift() {
        lift = new CANSparkMax(Constants.LiftConstants.LiftSparkmaxDeviceID, MotorType.kBrushless);
        lift.restoreFactoryDefaults();
        lift.setIdleMode(IdleMode.kBrake);
        lift.setSmartCurrentLimit(80);
        lift.setInverted(false);

        // m_Encoder = lift.getEncoder();
        lift.setSoftLimit(SoftLimitDirection.kForward, Constants.LiftConstants.Extended);
        lift.setSoftLimit(SoftLimitDirection.kReverse, Constants.LiftConstants.Home);
        lift.enableSoftLimit(SoftLimitDirection.kForward, true);
        lift.enableSoftLimit(SoftLimitDirection.kReverse, true);
        lift.burnFlash();
    }

    public void periodic() {
    }

    public void up() {
        lift.set(Constants.LiftConstants.up_speed);
    }

    public void down() {
        lift.set(Constants.LiftConstants.down_speed);
    }

}
