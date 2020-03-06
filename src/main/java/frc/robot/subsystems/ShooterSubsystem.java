/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.Shooter;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.Feeder;

/**
 * Shooter Subsystem, contains objects and methods needed to use and control the shooter & feeder
 */
public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_shooterLeft = new CANSparkMax(Shooter.kShooterLeftPort, MotorType.kBrushless);
    private final CANPIDController m_shooterPID = m_shooterLeft.getPIDController();
    private final CANEncoder m_shooterLeftEncoder = m_shooterLeft.getEncoder();

    private final CANSparkMax m_shooterRight = new CANSparkMax(Shooter.kShooterRightPort, MotorType.kBrushless);

    private final WPI_TalonSRX m_feeder = new WPI_TalonSRX(Feeder.kFeederTalonPort);

    private final DoubleSolenoid m_shooterSolenoid = new DoubleSolenoid(ShooterConstants.kShooterSolenoidPorts[0],
            ShooterConstants.kShooterSolenoidPorts[1]);

    private static boolean extended = false;

    public ShooterSubsystem() {
        m_shooterLeft.restoreFactoryDefaults();
        m_shooterRight.restoreFactoryDefaults();

        m_shooterRight.follow(m_shooterLeft, true);

        m_shooterPID.setP(Shooter.kP);
        m_shooterPID.setI(Shooter.kI);
        m_shooterPID.setD(Shooter.kD);
        m_shooterPID.setIZone(Shooter.kIz);
        m_shooterPID.setFF(Shooter.kF);
        m_shooterPID.setOutputRange(-1 * Shooter.kPeakOut, Shooter.kPeakOut);

        m_shooterPID.setFeedbackDevice(m_shooterLeftEncoder);

        /* Factory Default all hardware to prevent unexpected behaviour */
        m_feeder.configFactoryDefault();

        /* Config sensor used for Primary PID [Velocity] */
        m_feeder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Feeder.kPIDLoopIdx,
                Constants.kTimeoutMs);

        /**
         * Phase sensor accordingly. Positive Sensor Reading should match Green
         * (blinking) Leds on Talon
         */
        m_feeder.setSensorPhase(Feeder.kSensorPhase);

        /* Config the peak and nominal outputs */
        m_feeder.configNominalOutputForward(0, Constants.kTimeoutMs);
        m_feeder.configNominalOutputReverse(0, Constants.kTimeoutMs);
        m_feeder.configPeakOutputForward(1, Constants.kTimeoutMs);
        m_feeder.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        m_feeder.config_kF(Feeder.kPIDLoopIdx, Feeder.kF, Constants.kTimeoutMs);
        m_feeder.config_kP(Feeder.kPIDLoopIdx, Feeder.kP, Constants.kTimeoutMs);
        m_feeder.config_kI(Feeder.kPIDLoopIdx, Feeder.kI, Constants.kTimeoutMs);
        m_feeder.config_kD(Feeder.kPIDLoopIdx, Feeder.kD, Constants.kTimeoutMs);
    }

    public void runShooter() {
        m_shooterPID.setReference(Shooter.kRPM, ControlType.kVelocity);
    }

    public void stopShooter() {
        m_shooterPID.setReference(0, ControlType.kVelocity);
    }

    public void killShooter() {
        m_shooterLeft.pidWrite(0);
    }

    public void runFeeder() {
        m_feeder.set(ControlMode.Velocity, Feeder.kVelocity);
    }

    public void reverseFeeder() {
        m_feeder.set(ControlMode.Velocity, -1 * Feeder.kVelocity);
    }

    public void stopFeeder() {
        m_feeder.set(ControlMode.Velocity, 0);
    }

    public void killFeeder() {
        m_feeder.set(ControlMode.PercentOutput, 0);
    }

    public void extendShooter() {
        m_shooterSolenoid.set(Value.kForward);
    }

    public void retractShooter() {
        m_shooterSolenoid.set(Value.kReverse);
    }

    public void toggleExtended() {
        extended = !extended;
    }

    public boolean getExtended() {
        return extended;
    }
}