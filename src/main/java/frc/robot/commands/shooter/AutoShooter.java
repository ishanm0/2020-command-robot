/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.commands.drive.ArcadeDrive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Automatic shooting command - uses input from vision processing to determine
 * where to go to shoot
 */
public class AutoShooter extends StartEndCommand {
    private final ShooterSubsystem m_shooter;
    private final DriveSubsystem m_drive;

    /*
     * Get the entries within that table that correspond to the X and Y values for
     * some operation in your program.
     */
    private NetworkTableEntry xEntry = OIConstants.kTable.getEntry("x");
    private NetworkTableEntry yEntry = OIConstants.kTable.getEntry("Y");

    private double xPixelShift = xEntry.getDouble(0);
    private double yPixelShift = yEntry.getDouble(0);

    private final Joystick m_leftJoystick = OIConstants.joysticks[0];
    private final Joystick m_rightJoystick = OIConstants.joysticks[1];

    private final ADIS16470_IMU kIMU = DriveConstants.kIMU;

    /**
     * Creates a new AutoShooter.
     *
     * @param shooter The shooter subsystem this command will run on
     * @param drive The drive subsystem this command will use
     */
    public AutoShooter(ShooterSubsystem shooter, DriveSubsystem drive) {
        super(() -> new StartShooter(shooter), () -> new FinishShooter(shooter), shooter, drive);
        m_shooter = shooter;
        m_drive = drive;
        addRequirements(m_shooter, m_drive);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (m_shooter.getExtended()) {
            new RetractShooter(m_shooter);
        }
    }

    /**
     * Turns the xShift value into a DoubleSupplier
     * 
     * @return xShift scaled
     */
    public double xShift() {
        return (xPixelShift / ShooterConstants.kPixelWidth) * ShooterConstants.kXShiftFactor;
    }

    /**
     * Turns the yShift value into a DoubleSupplier
     * 
     * @return yShift scaled
     */
    public double yShift() {
        return (yPixelShift / ShooterConstants.kPixelHeight) * ShooterConstants.kYShiftFactor;
    }

    @Override
    public void execute() {
        xPixelShift = xEntry.getDouble(0);
        yPixelShift = yEntry.getDouble(0);

        if (Math.abs(xShift()) > 0) {
            new ArcadeDrive(m_drive, () -> {return 0;}, this::xShift);
        } else if (Math.abs(yShift()) > 0) {
            new ArcadeDrive(m_drive, this::yShift, () -> {return 0;});
        }
    }

    /**
     * Checks if driver has moved the joystick given in the X, Y, or Z axes
     * 
     * @param m_joystick the joystick to check input from
     * @return true if joystick is moved in any axis
     */
    public boolean driverMoved(Joystick m_joystick) {
        return m_joystick.getX() != 0 || m_joystick.getY() != 0 || m_joystick.getZ() != 0;
    }

    @Override
    public boolean isFinished() {
        return (driverMoved(m_leftJoystick) || driverMoved(m_rightJoystick))
                || (kIMU.getAccelInstantX() > ShooterConstants.kAccelThreshold
                        || kIMU.getAccelInstantY() > ShooterConstants.kAccelThreshold
                        || kIMU.getAccelInstantZ() > ShooterConstants.kAccelThreshold);
    }
}