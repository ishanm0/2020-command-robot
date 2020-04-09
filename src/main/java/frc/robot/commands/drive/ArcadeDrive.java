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
 * Drives the robot with y (linear) and z (rotation) values.
 */
public class ArcadeDrive extends CommandBase {
    private final DriveSubsystem m_drive;

    private DoubleSupplier y;
    private DoubleSupplier z;

    public ArcadeDrive(DriveSubsystem subsystem, DoubleSupplier ySupplier, DoubleSupplier zSupplier) {
        m_drive = subsystem;
        addRequirements(subsystem);

        y = ySupplier;
        z = zSupplier;
    }

    @Override
    public void execute() {
        m_drive.arcadeDrive(y.getAsDouble(), z.getAsDouble());
    }
}
