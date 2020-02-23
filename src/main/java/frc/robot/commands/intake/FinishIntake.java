/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives
 * backward.
 */
public class FinishIntake extends SequentialCommandGroup {
    /**
     * Creates a new FinishIntake.
     *
     * @param intake The intake subsystem this command will run on
     */
    public FinishIntake(IntakeSubsystem intake) {
        addCommands(
            // Pick up balls
            new StopIntake(intake),
            
            // Begin spinning the magazine wheels
            new StopMagazine(intake),
            
            // Lower the intake arm
            new RaiseIntake(intake));
    }
}