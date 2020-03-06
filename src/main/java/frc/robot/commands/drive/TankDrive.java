/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

/**
 * Drives the robot with l (left wheels) and r (right wheels) values.
 */
public class TankDrive extends CommandBase {
    private DriveSubsystem m_drive;

    private DoubleSupplier l;
    private DoubleSupplier r;

    public TankDrive(DriveSubsystem subsystem, DoubleSupplier lSupplier, DoubleSupplier rSupplier) {
        m_drive = subsystem;
        addRequirements(subsystem);

        l = lSupplier;
        r = rSupplier;
    }

    @Override
    public void execute() {
        m_drive.tankDrive(l.getAsDouble(), r.getAsDouble());
    }
}
