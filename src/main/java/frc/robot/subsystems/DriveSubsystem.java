/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.analog.adis16470.frc.ADIS16470_IMU.IMUAxis;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.triggers.ToggleButton;

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

    public boolean curvatureQuickTurn = false;
    private boolean reverseDrive = false;

    private final SendableChooser<String> m_driveModeChooser = new SendableChooser<>();
    private ArrayList<String> kDriveModeOptions = new ArrayList<String>();
    public int driveMode = DriveConstants.kDriveModeDefault;
    private String m_driveModeSelected;

    private NetworkTableEntry insanityFactorEntry = OIConstants.kTab.add("insanityFactor", insanityFactor)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    private NetworkTableEntry lSpeedEntry = OIConstants.kTab.add("lSpeed", 0).getEntry();
    private NetworkTableEntry rSpeedEntry = OIConstants.kTab.add("rSpeed", 0).getEntry();

    private NetworkTableEntry ySpeedEntry = OIConstants.kTab.add("ySpeed", 0).getEntry();
    private NetworkTableEntry zSpeedEntry = OIConstants.kTab.add("zSpeed", 0).getEntry();

    private ToggleButton curvatureQuickTurnButton = new ToggleButton("quickTurn", curvatureQuickTurn);
    private ToggleButton reverseDriveButton = new ToggleButton("reverseDrive", reverseDrive);

    private NetworkTableEntry leftEncoderEntry = OIConstants.kTab.add("leftEncoder", 0).getEntry();
    private NetworkTableEntry rightEncoderEntry = OIConstants.kTab.add("rightEncoder", 0).getEntry();

    public DriveSubsystem() {
        m_leftTalon1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, DriveConstants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        m_rightTalon1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, DriveConstants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        resetEncoders();

        OIConstants.kTab.add("m_leftTalon1", m_leftTalon1);
        OIConstants.kTab.add("m_rightTalon1", m_rightTalon1);

        kIMU.setYawAxis(IMUAxis.kZ);

        kDriveModeOptions.add("Tank");
        kDriveModeOptions.add("Arcade");
        kDriveModeOptions.add("Curvature");

        m_driveModeChooser.setDefaultOption(kDriveModeOptions.get(driveMode), kDriveModeOptions.get(driveMode));

        for (int i = 0; i < kDriveModeOptions.size(); i++) {
            if (i != driveMode) {
                m_driveModeChooser.addOption(kDriveModeOptions.get(i), kDriveModeOptions.get(i));
            }
        }

        OIConstants.kTab.add(m_driveModeChooser);
    }

    @Override
    public void periodic() {
        insanityFactor = insanityFactorEntry.getDouble(insanityFactor);
        reverseDrive = reverseDriveButton.get();

        leftEncoderEntry.forceSetNumber(m_leftTalon1.getSelectedSensorVelocity());
        rightEncoderEntry.forceSetNumber(m_rightTalon1.getSelectedSensorVelocity());

        m_driveModeSelected = m_driveModeChooser.getSelected();
        driveMode = kDriveModeOptions.indexOf(m_driveModeSelected);
        if (driveMode < 0 || driveMode >= kDriveModeOptions.size()) {
            driveMode = DriveConstants.kDriveModeDefault;
        }
    }

    /**
     * Drives the robot at given left/right speeds. Speeds range in [-1, 1].
     * 
     * @param leftSpeed  speed for left wheels
     * @param rightSpeed speed for right wheels
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(-1 * insanityFactor * leftSpeed, -1 * insanityFactor * rightSpeed, false);
    }

    /**
     * Drives the robot at given y/z (rotation) speeds. Speeds range in [-1, 1].
     * 
     * @param ySpeed speed in forward/backward direction
     * @param zSpeed rotational speed
     */
    public void arcadeDrive(double ySpeed, double zSpeed) {
        m_drive.arcadeDrive(insanityFactor * ySpeed, insanityFactor * zSpeed, false);
        // TODO: does arcade zSpeed need insanityFactor?
    }

    /**
     * Drives the robot at given y/z (rotation) speeds. Speeds range in [1, 1].
     * 
     * @param ySpeed      spead in forward/backward direction
     * @param zSpeed      controls curvature of robot's path, instead of direct
     *                    rotational speed
     * @param isQuickTurn if true, zSpeed controls direct rotational speed (like
     *                    arcadeDrive). If false, typical curvatureDrive
     */
    public void curvatureDrive(double ySpeed, double zSpeed, boolean isQuickTurn) {
        m_drive.curvatureDrive(insanityFactor * ySpeed, insanityFactor * zSpeed, isQuickTurn);
        // TODO: does curvature zSpeed need insanityFactor?
    }

    /**
     * Alternately uses tank drive or arcade drive depending on dashboard value.
     * 
     * @param lSpeed tank drive left speed
     * @param rSpeed tank drive right speed
     * @param ySpeed arcade/curvature drive linear speed
     * @param zSpeed arcade/curvature drive rotation speed
     */
    public void drive(double lSpeed, double rSpeed, double ySpeed, double zSpeed) {
        curvatureQuickTurn = curvatureQuickTurnButton.get();
        
        if (reverseDrive) {
            switch (driveMode) {
                case 0:
                    tankDrive(-rSpeed, -lSpeed);
                    break;
                case 1:
                    arcadeDrive(-ySpeed, zSpeed);
                    break;
                case 2:
                    curvatureDrive(-ySpeed, zSpeed, curvatureQuickTurn);
                    break;
            }
        } else {
            switch (driveMode) {
                case 0:
                    tankDrive(lSpeed, rSpeed);
                    break;
                case 1:
                    arcadeDrive(ySpeed, zSpeed);
                    break;
                case 2:
                    curvatureDrive(ySpeed, zSpeed, curvatureQuickTurn);
                    break;
            }
        }

        lSpeedEntry.forceSetDouble(lSpeed);
        rSpeedEntry.forceSetDouble(rSpeed);

        ySpeedEntry.forceSetDouble(ySpeed);
        zSpeedEntry.forceSetDouble(zSpeed);
    }

    /**
     * Set whether robot quickTurns in curvature drive
     * 
     * @param input if true, curvature zSpeed directly controls rotation; if false,
     *              curvature zSpeed controls curvature of robot's path
     */
    public void setCurvatureQuickTurn(boolean input) {
        curvatureQuickTurn = input;
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