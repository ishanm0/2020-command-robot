/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.trench;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.TrenchConstants;
import frc.robot.subsystems.TrenchSubsystem;


/**
 * Control Panel/Trench Rotation control (PID on talon)
 */
public class Rotation extends CommandBase {
    private TrenchSubsystem m_trench;

    private Color setColor;

    private int colorCount;
    private int tempCount;

    private int countThreshold = TrenchConstants.kCountThreshold;
    private double confidenceThreshold = TrenchConstants.kConfidenceThreshold;
    private int maxRotations = TrenchConstants.kMaxRotations;

    // private double rotationSpeed = TrenchConstants.kRotationSpeed;

    private ColorMatch colorMatcher;
    private ColorSensorV3 colorSensor;

    private WPI_TalonSRX m_trenchTalon;

    private boolean complete = false;

    public Rotation(TrenchSubsystem subsystem) {
        m_trench = subsystem;
        addRequirements(subsystem);

        colorMatcher = m_trench.getColorMatcher();
        colorSensor = m_trench.getColorSensor();

        m_trenchTalon = m_trench.getTrenchTalon();
    }

    @Override
    public void initialize() {
        Color detectedColor = colorSensor.getColor();

        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        setColor = match.color;

        colorCount = 0;
        tempCount = 0;

        /* Factory Default all hardware to prevent unexpected behaviour */
        m_trenchTalon.configFactoryDefault();

        /* Config sensor used for Primary PID [Velocity] */
        m_trenchTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TrenchConstants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        /**
         * Phase sensor accordingly. Positive Sensor Reading should match Green
         * (blinking) Leds on Talon
         */
        m_trenchTalon.setSensorPhase(TrenchConstants.kSensorPhase);

        /* Config the peak and nominal outputs */
        m_trenchTalon.configNominalOutputForward(0, Constants.kTimeoutMs);
        m_trenchTalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        m_trenchTalon.configPeakOutputForward(1, Constants.kTimeoutMs);
        m_trenchTalon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
		m_trenchTalon.config_kF(TrenchConstants.kPIDLoopIdx, TrenchConstants.kF, Constants.kTimeoutMs);
		m_trenchTalon.config_kP(TrenchConstants.kPIDLoopIdx, TrenchConstants.kP, Constants.kTimeoutMs);
		m_trenchTalon.config_kI(TrenchConstants.kPIDLoopIdx, TrenchConstants.kI, Constants.kTimeoutMs);
		m_trenchTalon.config_kD(TrenchConstants.kPIDLoopIdx, TrenchConstants.kD, Constants.kTimeoutMs);
    }

    @Override
    public void execute() {
        Color newColor = colorSensor.getColor();

        ColorMatchResult match = colorMatcher.matchClosestColor(newColor);

        boolean colorMatched = match.color == setColor;

        if (colorCount < maxRotations * 2) {
            // m_trenchTalon.set(rotationSpeed);
            m_trenchTalon.set(ControlMode.Velocity, TrenchConstants.kVelocity);
        } else {
            m_trenchTalon.set(ControlMode.PercentOutput, 0);

            complete = true;
        }

        if (colorMatched && match.confidence > confidenceThreshold) {
            tempCount++;
        } else if (tempCount > countThreshold) {
            colorCount++;
            tempCount = 0;
        } else {
            tempCount = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}
