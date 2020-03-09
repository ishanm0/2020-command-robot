/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private final CANEncoder m_shooterLeftEncoder = m_shooterLeft.getEncoder();
    
    private final CANSparkMax m_shooterRight = new CANSparkMax(Shooter.kShooterRightPort, MotorType.kBrushless);
    private final CANEncoder m_shooterRightEncoder = m_shooterRight.getEncoder();

    private final WPI_TalonSRX m_feeder = new WPI_TalonSRX(Feeder.kFeederTalonPort);

    private final DoubleSolenoid m_shooterSolenoid = new DoubleSolenoid(ShooterConstants.kShooterSolenoidPorts[0],
            ShooterConstants.kShooterSolenoidPorts[1]);

    private static boolean extended = false;

    private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
    private NetworkTableEntry leftEncoderEntry = tab.add("leftShooterEncoder", 0).getEntry();
    private NetworkTableEntry rightEncoderEntry = tab.add("rightShooterEncoder", 0).getEntry();

    public ShooterSubsystem() {
        m_shooterLeft.restoreFactoryDefaults();
        m_shooterRight.restoreFactoryDefaults();

        m_shooterRight.follow(m_shooterLeft, true);

        /* Factory Default all hardware to prevent unexpected behaviour */
        m_feeder.configFactoryDefault();

        /* Config sensor used for Primary PID [Velocity] */
        m_feeder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Feeder.kPIDLoopIdx,
                Constants.kTimeoutMs);

        m_feeder.setSensorPhase(Feeder.kSensorPhase);
    }

    @Override
    public void periodic() {
        leftEncoderEntry.forceSetNumber(m_shooterLeftEncoder.getVelocity());
        rightEncoderEntry.forceSetNumber(m_shooterRightEncoder.getVelocity());
    }

    public void runShooter() {
        m_shooterLeft.set(Shooter.kSpeed);
    }

    public void stopShooter() {
        m_shooterLeft.set(0);
    }

    public void killShooter() {
        m_shooterLeft.set(0);
    }

    public void runFeeder() {
        m_feeder.set(TalonSRXControlMode.PercentOutput, Feeder.kSpeed);
    }

    public void reverseFeeder() {
        m_feeder.set(TalonSRXControlMode.PercentOutput, -1 * Feeder.kSpeed);
    }

    public void stopFeeder() {
        m_feeder.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void killFeeder() {
        m_feeder.set(TalonSRXControlMode.PercentOutput, 0);
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