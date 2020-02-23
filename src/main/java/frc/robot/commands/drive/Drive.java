/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

/**
 * An example command that uses an example subsystem.
 */
public class Drive extends ConditionalCommand {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    /**
     * Creates a new Drive command.
     *
     * @param drive The subsystem used by this command.
     */
    public Drive(DriveSubsystem drive, Joystick m_leftJoystick, Joystick m_rightJoystick) {
        super(new TankDrive(drive, m_leftJoystick, m_rightJoystick),
                new ArcadeDrive(drive, m_leftJoystick, m_rightJoystick), drive::getTank);
    }
}
