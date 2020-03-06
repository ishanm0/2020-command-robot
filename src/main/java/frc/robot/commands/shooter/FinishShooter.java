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
 * Runs the ending shooter sequence: stops the feeder wheel, stops the shooter
 * wheel
 */
public class FinishShooter extends SequentialCommandGroup {
    /**
     * Creates a new FinishShooter.
     *
     * @param shooter The shooter subsystem this command will run on
     */
    public FinishShooter(ShooterSubsystem shooter) {
        addCommands(
                // Stop running the feeder wheel
                new StopFeeder(shooter),

                // Stop running the shooter wheel
                new StopShooter(shooter));
    }
}