/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

/**
 * A command that releases the hatch.
 */
public class ArcadeDrive extends CommandBase {
    private DriveSubsystem m_drive;

    private Joystick m_leftJoystick;
    private Joystick m_rightJoystick;

    public ArcadeDrive(DriveSubsystem subsystem, Joystick m_leftJoystick, Joystick m_rightJoystick) {
        m_drive = subsystem;
        addRequirements(subsystem);

        this.m_leftJoystick = m_leftJoystick;
        this.m_rightJoystick = m_rightJoystick;
    }

    @Override
    public void execute() {
        m_drive.arcadeDrive(m_leftJoystick.getY(), m_rightJoystick.getX());
    }
}
