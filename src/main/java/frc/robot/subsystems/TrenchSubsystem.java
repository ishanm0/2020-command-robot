/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TrenchConstants;

/**
 * Trench Subsystem, contains objects and methods needed to use and control the trench/control panel mechanism
 */
public class TrenchSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_trenchTalon = new WPI_TalonSRX(TrenchConstants.kTrenchTalonPort);

    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch colorMatcher = new ColorMatch();

    /*
     * private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
     * private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
     * private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
     * private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524,
     * 0.113);
     */

    private final Color kBlueTarget = ColorMatch.makeColor(0.224, 0.476, 0.300);
    private final Color kGreenTarget = ColorMatch.makeColor(0.209, 0.565, 0.226);
    private final Color kRedTarget = ColorMatch.makeColor(0.526, 0.341, 0.129);
    private final Color kYellowTarget = ColorMatch.makeColor(0.337, 0.528, 0.113);

    /**
     * Creates a new TrenchSubsystem.
     */
    public TrenchSubsystem() {
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget);

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
    }

    public ColorMatch getColorMatcher() {
        return colorMatcher;
    }

    public ColorSensorV3 getColorSensor() {
        return colorSensor;
    }

    public WPI_TalonSRX getTrenchTalon() {
        return m_trenchTalon;
    }

    public double getTrenchTalonRate() {
        return m_trenchTalon.getSelectedSensorVelocity();
    }

    public int getColor() {
        Color detectedColor = colorSensor.getColor();

        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            return 0;
        } else if (match.color == kGreenTarget) {
            return 1;
        } else if (match.color == kRedTarget) {
            return 2;
        } else if (match.color == kYellowTarget) {
            return 3;
        } else {
            return -1;
        }
    }
}