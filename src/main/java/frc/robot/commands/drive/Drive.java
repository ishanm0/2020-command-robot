/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.DriveSubsystem;

/**
 * Runs tank drive or arcade drive depending on boolean input value
 */
public class Drive extends CommandBase {
    private DriveSubsystem m_drive;

    private double l;
    private double r;

    private double y;
    private double z;

    private static final String[] kAxisOptions = {"Left X", "Left Y", "Left Z", "Right X", "Right Y", "Right Z"};

    private String m_tankLSelected;
    private String m_tankRSelected;
    
    private String m_arcadeYSelected;
    private String m_arcadeZSelected;
    
    private final SendableChooser<String> m_tankLChooser = new SendableChooser<>();
    private final SendableChooser<String> m_tankRChooser = new SendableChooser<>();
    
    private final SendableChooser<String> m_arcadeYChooser = new SendableChooser<>();
    private final SendableChooser<String> m_arcadeZChooser = new SendableChooser<>();

    private final int kTankLDefault = 1;
    private final int kTankRDefault = 4;
    
    private final int kArcadeYDefault = 4;
    private final int kArcadeZDefault = 2;

    private final Joystick m_leftJoystick = OIConstants.joysticks[0];
    private final Joystick m_rightJoystick = OIConstants.joysticks[1];

    /**
     * Creates a new Drive command.
     *
     * @param drive The subsystem used by this command.
     */
    public Drive(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive);

        m_tankLChooser.setDefaultOption(kAxisOptions[kTankLDefault], kAxisOptions[kTankLDefault]);
        m_tankRChooser.setDefaultOption(kAxisOptions[kTankRDefault], kAxisOptions[kTankRDefault]);

        m_arcadeYChooser.setDefaultOption(kAxisOptions[kArcadeYDefault], kAxisOptions[kArcadeYDefault]);
        m_arcadeZChooser.setDefaultOption(kAxisOptions[kArcadeZDefault], kAxisOptions[kArcadeZDefault]);

        for (int i = 0; i < kAxisOptions.length; i++) {
            if (i != kTankLDefault) {
                m_tankLChooser.addOption(kAxisOptions[i], kAxisOptions[i]);
            }

            if (i != kTankRDefault) {
                m_tankRChooser.addOption(kAxisOptions[i], kAxisOptions[i]);
            }

            if (i != kArcadeYDefault) {
                m_arcadeYChooser.addOption(kAxisOptions[i], kAxisOptions[i]);
            }

            if (i != kArcadeZDefault) {
                m_arcadeZChooser.addOption(kAxisOptions[i], kAxisOptions[i]);
            }
        }

        Shuffleboard.getTab("SmartDashboard").add(m_tankLChooser);
        Shuffleboard.getTab("SmartDashboard").add(m_tankRChooser);

        Shuffleboard.getTab("SmartDashboard").add(m_arcadeYChooser);
        Shuffleboard.getTab("SmartDashboard").add(m_arcadeZChooser);
    }

    @Override
    public void execute() {
        m_tankLSelected = m_tankLChooser.getSelected();
        m_tankRSelected = m_tankRChooser.getSelected();

        m_arcadeYSelected = m_arcadeYChooser.getSelected();
        m_arcadeZSelected = m_arcadeZChooser.getSelected();

        l = getSelectedJoystickValue(m_tankLSelected);
        r = getSelectedJoystickValue(m_tankRSelected);

        y = getSelectedJoystickValue(m_arcadeYSelected);
        z = getSelectedJoystickValue(m_arcadeZSelected);

        m_drive.drive(l, r, y, z);
    }

    public double getSelectedJoystickValue(String selected) {
        switch (selected) {
            case "Left X":
                return m_leftJoystick.getX();
            case "Left Y":
                return m_leftJoystick.getY();
            case "Left Z":
                return m_leftJoystick.getZ();
            case "Right X":
                return m_rightJoystick.getX();
            case "Right Y":
                return m_rightJoystick.getY();
            case "Right Z":
                return m_rightJoystick.getZ();
            default:
                return 0; 
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
