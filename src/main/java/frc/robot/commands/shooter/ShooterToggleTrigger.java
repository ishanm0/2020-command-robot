/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;

/**
 * Trigger to switch shooting modes - DPad down
 */
public class ShooterToggleTrigger extends Trigger {
    @Override
    public boolean get() {
        return OIConstants.joysticks[OIConstants.kShooterToggle[0]].getPOV(0) <= OIConstants.kShooterToggle[2]
                && OIConstants.joysticks[OIConstants.kShooterToggle[0]].getPOV(0) >= OIConstants.kShooterToggle[1];
    }
}