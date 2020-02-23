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
public class StartIntake extends SequentialCommandGroup {
    /**
     * Creates a new StartIntake.
     *
     * @param intake The intake subsystem this command will run on
     */
    public StartIntake(IntakeSubsystem intake) {
        addCommands(
            // Lower the intake arm
            new LowerIntake(intake), 
            
            // Begin spinning the magazine wheels
            new RunMagazine(intake), 
            
            // Pick up balls
            new RunIntake(intake));
    }
}