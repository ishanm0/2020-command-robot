/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Runs the magazine wheels in reverse
 */
public class ReverseMagazine extends InstantCommand {
    public ReverseMagazine(IntakeSubsystem subsystem) {
        super(subsystem::reverseMagazine, subsystem);
    }
}
