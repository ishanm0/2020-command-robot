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
 * Runs the ending intake sequence: stops the outer intake wheels, stops the
 * throat wheels, stops the magazine, and raises the intake arm
 */
public class FinishIntake extends SequentialCommandGroup {
    /**
     * Creates a new FinishIntake.
     *
     * @param intake The intake subsystem this command will run on
     */
    public FinishIntake(IntakeSubsystem intake) {
        addRequirements(intake);
        addCommands(
                // Stop intake outer wheels
                new StopIntakeOuter(intake),

                // Stop throat wheels
                new StopThroat(intake),

                // Stop spinning the magazine wheels
                new StopMagazine(intake),

                // Raise the intake arm
                new RaiseIntake(intake));
    }
}