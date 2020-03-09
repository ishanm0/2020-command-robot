/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.analog.adis16470.frc.ADIS16470_IMU;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.*;

/**
 * Drive Subsystem, contains objects and methods needed to drive the robot
 */
public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_leftTalon1 = new WPI_TalonSRX(DriveConstants.kLeftTalon1Port);
    private final WPI_TalonSRX m_leftTalon2 = new WPI_TalonSRX(DriveConstants.kLeftTalon2Port);
    private final WPI_TalonSRX m_leftTalon3 = new WPI_TalonSRX(DriveConstants.kLeftTalon3Port);

    private final WPI_TalonSRX m_rightTalon1 = new WPI_TalonSRX(DriveConstants.kRightTalon1Port);
    private final WPI_TalonSRX m_rightTalon2 = new WPI_TalonSRX(DriveConstants.kRightTalon2Port);
    private final WPI_TalonSRX m_rightTalon3 = new WPI_TalonSRX(DriveConstants.kRightTalon3Port);

    private final SpeedControllerGroup m_leftDrive = new SpeedControllerGroup(m_leftTalon1, m_leftTalon2, m_leftTalon3);
    private final SpeedControllerGroup m_rightDrive = new SpeedControllerGroup(m_rightTalon1, m_rightTalon2,
            m_rightTalon3);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);

    private final ADIS16470_IMU kIMU = DriveConstants.kIMU;

    private double insanityFactor = 0.5;

    public static boolean tank = DriveConstants.kTankDefault;

    private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
    private NetworkTableEntry insanityFactorEntry = tab.add("insanityFactor", insanityFactor)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();
    private NetworkTableEntry driveToggleEntry = tab.add("driveToggle", tank)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    private NetworkTableEntry lSpeedEntry = tab.add("lSpeed", 0).getEntry();
    private NetworkTableEntry rSpeedEntry = tab.add("rSpeed", 0).getEntry();
    
    private NetworkTableEntry ySpeedEntry = tab.add("ySpeed", 0).getEntry();
    private NetworkTableEntry zSpeedEntry = tab.add("zSpeed", 0).getEntry();
    
    private NetworkTableEntry leftEncoderEntry = tab.add("leftEncoder", 0).getEntry();
    private NetworkTableEntry rightEncoderEntry = tab.add("rightEncoder", 0).getEntry();

    public DriveSubsystem() {
        m_leftTalon1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, DriveConstants.kPIDLoopIdx, Constants.kTimeoutMs);
        m_rightTalon1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, DriveConstants.kPIDLoopIdx, Constants.kTimeoutMs);

        resetEncoders();

        tab.add("m_leftTalon1", m_leftTalon1);
        tab.add("m_rightTalon1", m_rightTalon1);
    }

    @Override
    public void periodic() {
        insanityFactor = insanityFactorEntry.getDouble(insanityFactor);
        tank = driveToggleEntry.getBoolean(tank);

        leftEncoderEntry.forceSetNumber(m_leftTalon1.getSelectedSensorVelocity());
        rightEncoderEntry.forceSetNumber(m_rightTalon1.getSelectedSensorVelocity());
    }
    
    /**
     * Drives the robot at given left/right speeds. Speeds range from [-1, 1].
     * 
     * @param leftSpeed speed for left wheels
     * @param rightSpeed speed for right wheels
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(-1 * insanityFactor * leftSpeed, -1 * insanityFactor * rightSpeed, false);
    }

    /**
     * Drives the robot at given y/z (rotation) speeds. Speeds range from [-1, 1].
     * 
     * @param ySpeed speed in forward/backward direction
     * @param zSpeed rotational speed
     */
    public void arcadeDrive(double ySpeed, double zSpeed) {
        m_drive.arcadeDrive(insanityFactor * ySpeed, insanityFactor * zSpeed, false);
    }

    /**
     * Alternately uses tank drive or arcade drive depending on dashboard value.
     * 
     * @param lSpeed tank drive left speed
     * @param rSpeed tank drive right speed
     * @param ySpeed arcade drive linear speed
     * @param zSpeed arcade drive rotation speed
     */
    public void drive(double lSpeed, double rSpeed, double ySpeed, double zSpeed) {
        if (getTank()) {
            tankDrive(lSpeed, rSpeed);
        } else {
            arcadeDrive(ySpeed, zSpeed);
        }

        lSpeedEntry.forceSetDouble(lSpeed);
        rSpeedEntry.forceSetDouble(rSpeed);
        ySpeedEntry.forceSetDouble(ySpeed);
        zSpeedEntry.forceSetDouble(zSpeed);
    }

    /**
     * @return the value of the tank/arcade boolean (tank if true, arcade if false)
     */
    public boolean getTank() {
        return tank;
    }
    
    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        kIMU.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(kIMU.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return kIMU.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftTalon1.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
        m_rightTalon1.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
    }
}