/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.trench;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.TrenchConstants;
import frc.robot.subsystems.TrenchSubsystem;

public class Position extends CommandBase {
    private TrenchSubsystem m_trench;

    private double positionSpeed = TrenchConstants.kPositionSpeed;

    private WPI_TalonSRX m_trenchTalon;

    private int positionInt = -1;
    private int colorInt = -1;

    private boolean complete;

    public Position(TrenchSubsystem subsystem) {
        m_trench = subsystem;
        addRequirements(subsystem);

        m_trenchTalon = m_trench.getTrenchTalon();
    }

    @Override
    public void initialize() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();

        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    positionInt = 0;
                    break;
                case 'G':
                    positionInt = 1;
                    break;
                case 'R':
                    positionInt = 2;
                    break;
                case 'Y':
                    positionInt = 3;
                    break;
                default:
                    new PrintCommand("Ishan can't code, so colors are weird");
                    break;
            }

            positionInt += 2;
            if (positionInt > 3) {
                positionInt -= 4;
            }
        }
    }

    @Override
    public void execute() {
        if (positionInt > -1) {
            colorInt = m_trench.getColor();

            if (colorInt < positionInt) {
                m_trenchTalon.set(-1 * positionSpeed);
            } else if (colorInt > positionInt) {
                m_trenchTalon.set(positionSpeed);
            } else {
                m_trenchTalon.set(0);

                complete = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}
