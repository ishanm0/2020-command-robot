/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import frc.robot.subsystems.DriveSubsystem;

/**
 * Runs tank drive or arcade drive depending on boolean input value
 */
public class Drive extends ConditionalCommand {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    /**
     * Creates a new Drive command.
     *
     * @param drive The subsystem used by this command.
     */
    public Drive(DriveSubsystem drive, Joystick m_leftJoystick, Joystick m_rightJoystick) {
        super(new TankDrive(drive, m_leftJoystick::getY, m_rightJoystick::getY),
                new ArcadeDrive(drive, m_leftJoystick::getY, m_rightJoystick::getX), drive::getTank);
    }
}
