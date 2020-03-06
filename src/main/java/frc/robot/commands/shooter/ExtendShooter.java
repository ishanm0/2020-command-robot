/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ShooterSubsystem;

/**
 * Extends the shooter solenoid (moves shooter to upward position)
 */
public class ExtendShooter extends InstantCommand {
    public ExtendShooter(ShooterSubsystem subsystem) {
        super(subsystem::extendShooter, subsystem);
    }
}
