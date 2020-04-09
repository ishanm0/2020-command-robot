/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Limit Switch Trigger
 */
public class LimitSwitch extends Trigger {
    DigitalInput limitSwitch;

    public LimitSwitch(int switchID) {
        limitSwitch = new DigitalInput(switchID);
    }

    @Override
    public boolean get() {
        return limitSwitch.get();
    }
}