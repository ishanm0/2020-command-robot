/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.DriveSubsystem;

/**
 * Runs tank drive or arcade drive depending on boolean input value
 */
public class Drive extends CommandBase {
    private final DriveSubsystem m_drive;

    private double l;
    private double r;

    private double y;
    private double z;

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
    }

    @Override
    public void execute() {
        l = m_leftJoystick.getY();
        r = m_rightJoystick.getY();

        y = m_leftJoystick.getY();
        z = m_rightJoystick.getZ();

        m_drive.drive(l, r, y, z);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
