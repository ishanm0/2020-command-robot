/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import frc.robot.subsystems.ShooterSubsystem;

/**
 * Switches shooting modes from close to far and vice versa depending on
 * existing mode value
 */
public class ToggleShooter extends ConditionalCommand {
    public ToggleShooter(ShooterSubsystem subsystem) {
        super(new RetractShooter(subsystem), new ExtendShooter(subsystem), subsystem::getExtended);
        subsystem.toggleExtended();
    }
}
