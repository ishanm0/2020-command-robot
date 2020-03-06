/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Constants.OIConstants;

import frc.robot.commands.drive.Drive;
import frc.robot.commands.intake.FinishIntake;
import frc.robot.commands.intake.StartIntake;

import frc.robot.commands.shooter.AutoShooter;
import frc.robot.commands.shooter.FinishShooter;
import frc.robot.commands.shooter.ShooterToggleTrigger;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.ToggleShooter;
import frc.robot.commands.trench.Position;
import frc.robot.commands.trench.Rotation;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TrenchSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final TrenchSubsystem m_trench = new TrenchSubsystem();

    // The default driving command
    private final Drive m_driveCommand = new Drive(m_drive, OIConstants.joysticks[0], OIConstants.joysticks[1]);

    private AutoShooter m_shooterCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // set the default drive command to split-stick tank drive
        m_drive.setDefaultCommand(m_driveCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /*
         * new JoystickButton(OIConstants.joysticks[OIConstants.kDriveToggle[0]],
         * OIConstants.kDriveToggle[1]) .whenPressed(() -> m_drive.toggleMode());
         */

        new JoystickButton(OIConstants.joysticks[OIConstants.kIntakeControl[0]], OIConstants.kIntakeControl[1])
                .whenPressed(() -> new StartIntake(m_intake))
                .whenReleased(() -> new FinishIntake(m_intake));

        new JoystickButton(OIConstants.joysticks[OIConstants.kAutoShooterControl[0]],
                OIConstants.kAutoShooterControl[1])
                .whenPressed(m_shooterCommand = new AutoShooter(m_shooter, m_drive))
                .whenReleased(() -> m_shooterCommand.end(false));

        new JoystickButton(OIConstants.joysticks[OIConstants.kManualShooterControl[0]], OIConstants.kManualShooterControl[1])
                .whenPressed(() -> new StartShooter(m_shooter))
                .whenReleased(() -> new FinishShooter(m_shooter));

        new ShooterToggleTrigger()
                .whenActive(() -> new ToggleShooter(m_shooter));
        /*
         * new JoystickButton(joysticks[OIConstants.kStartIntake[0]],
         * OIConstants.kStartIntake[1]) .whenPressed(() -> new StartIntake(m_intake));
         * 
         * new JoystickButton(joysticks[OIConstants.kStopIntake[0]],
         * OIConstants.kStopIntake[1]) .whenPressed(() -> new StopIntake(m_intake));
         */

        /*
         * new JoystickButton(joysticks[OIConstants.kRotation[0]],
         * OIConstants.kRotation[1]) .whenPressed(() -> new Rotation(m_trench));
         * 
         * new JoystickButton(joysticks[OIConstants.kPosition[0]],
         * OIConstants.kPosition[1]) .whenPressed(() -> new Position(m_trench));
         */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
