/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ShooterSubsystem;

/**
 * Runs the starting shooter sequence: starts the shooter wheel, starts the
 * feeder wheel
 */
public class StartShooter extends SequentialCommandGroup {
    /**
     * Creates a new StartShooter.
     *
     * @param shooter The shooter subsystem this command will run on
     */
    public StartShooter(ShooterSubsystem shooter) {
        addRequirements(shooter);
        addCommands(
                // Begin running the shooter wheel
                new RunShooter(shooter),

                // Begin running the feeder wheel
                new RunFeeder(shooter));
    }
}