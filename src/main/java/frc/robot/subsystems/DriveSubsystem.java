/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.analog.adis16470.frc.ADIS16470_IMU;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

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

    private final ADIS16470_IMU m_imu = new ADIS16470_IMU();

    private double insanityFactor = 0.5;
    // private double oldThrottle = 0;

    public static boolean tank = false;

    private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
    private NetworkTableEntry insanityFactorEntry = tab.add("insanityFactor", insanityFactor)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();

    public DriveSubsystem() {
        
    }
    
    /**
     * Drives the robot at given left/right speeds. Speeds range from [-1, 1].
     * 
     * @param leftSpeed speed for left wheels
     * @param rightSpeed speed for right wheels
     */
    // @SuppressWarnings("ParameterName")
    public void tankDrive(double leftSpeed, double rightSpeed) {
        insanityFactor = insanityFactorEntry.getDouble(insanityFactor);

        m_drive.tankDrive(insanityFactor * leftSpeed, insanityFactor * rightSpeed);
    }

    /**
     * Drives the robot at given y/z (rotation) speeds. Speeds range from [-1, 1].
     * 
     * @param ySpeed speed in forward/backward direction
     * @param zSpeed rotational speed
     */
    // @SuppressWarnings("ParameterName")
    public void arcadeDrive(double ySpeed, double zSpeed) {
        insanityFactor = insanityFactorEntry.getDouble(insanityFactor);

        m_drive.arcadeDrive(insanityFactor * ySpeed, insanityFactor * zSpeed);
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /**
     * Switch between tank and arcade driving
     */
    public void toggleMode() {
        tank = !tank;
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
        m_imu.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(m_imu.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_imu.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
