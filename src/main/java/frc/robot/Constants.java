/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftTalon1Port = 36;
        public static final int kLeftTalon2Port = 35;
        public static final int kLeftTalon3Port = 34;

        public static final int kRightTalon1Port = 24;
        public static final int kRightTalon2Port = 23;
        public static final int kRightTalon3Port = 22;

        // TODO: set correct track width (inches)
        public static final double kTrackWidth = 20.0;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                Units.inchesToMeters(kTrackWidth));

        public static final boolean kGyroReversed = false;

        public static final ADIS16470_IMU kIMU = new ADIS16470_IMU();

        public static final double kAccelThreshold = 0.05;

        public static final int kMaxRPM = 3600;

        public static final boolean kLeftInverted = false;
        public static final boolean kRightInverted = false;

        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
         * or 3. Only the first two (0,1) are visible in web-based configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
         * we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        // public final static double kVelocity = kMaxRPM * 4096 / 600;
        public final static double kVelocity = 3600;

        public final static double kP = 0;
        public final static double kI = 0;
        public final static double kD = 0;
        public final static double kF = 1023.0 / 7200.0;
        public final static int kIz = 300;
        public final static double kPeakOut = 1.00;

        public final static boolean kSensorPhase = true;
    }

    public static final class OIConstants {
        private static final int kLeftJoystickPort = 0;
        private static final int kRightJoystickPort = 1;

        // The joysticks
        private static final Joystick kLeftJoystick = new Joystick(/* OIConstants. */kLeftJoystickPort);
        private static final Joystick kRightJoystick = new Joystick(/* OIConstants. */kRightJoystickPort);

        public static final Joystick[] joysticks = { kLeftJoystick, kRightJoystick };

        public static final int[] kIntakeControl = { 0, 1 }; // left joystick, trigger

        public static final int[] kAutoShooterControl = { 1, 1 }; // right joystick, trigger
        public static final int[] kManualShooterControl = { 1, 2 }; // right joystick, thumb side button

        public static final int kToggleShooterStick = 1;
        public static final int kRunShooterStick = 1;
        public static final int kStopShooterStick = 1;
        public static final int kKillShooterStick = 1;

        public static final int kRunFeederStick = 0;
        public static final int kStopFeederStick = 0;
        public static final int kKillFeederStick = 0;
        public static final int kReverseFeederStick = 0;

        public static final int[] kRotation = { 0, 3 }; // left joystick, bottom left thumb button
        public static final int[] kPosition = { 0, 4 }; // left joystick, bottom right thumb button
    }

    public static final class IntakeConstants {
        public static final int[] kIntakeSolenoidPorts = { 0, 1 };

        public static final int kIntakeTalonOuterPort = 4;
        public static final int kIntakeTalonFrontPort = 5;
        public static final int kIntakeTalonBackPort = 6;

        public static final int kMagazineTalonLeftPort = 10;
        public static final int kMagazineTalonRightPort = 11;

        public static final double kIntakeOuterSpeed = 0.5;

        public static final double kIntakeFlatSpeed = 0.5;

        public static final double kMagazineSpeed = 0.5;
    }

    public static final class ShooterConstants {
        public static final class Feeder {
            public static final int kFeederTalonPort = 12;
            /**
             * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
             * or 3. Only the first two (0,1) are visible in web-based configuration.
             */
            public static final int kSlotIdx = 0;

            /**
             * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
             * we just want the primary one.
             */
            public static final int kPIDLoopIdx = 0;

            public final static double kRPM = 500.0;

            public final static double kVelocity = kRPM * 4096 / 600;

            public final static double kP = 0.25;
            public final static double kI = 0.001;
            public final static double kD = 20.0;
            public final static double kF = 1023.0 / 7200.0;
            public final static int kIz = 300;
            public final static double kPeakOut = 1.00;

            public final static boolean kSensorPhase = true;
        }

        public static final class Shooter {
            public static final int kShooterLeftPort = 13;
            public static final int kShooterRightPort = 14;

            public final static double kRPM = 500.0;

            public final static double kP = 0.25;
            public final static double kI = 0.001;
            public final static double kD = 20.0;
            public final static double kF = 1023.0 / 7200.0;
            public final static int kIz = 300;
            public final static double kPeakOut = 1.00;

            public final static double kMaxTurnSpeed = 0.5;

            public final static int kPixelWidth = 160;
            public final static int kPixelHeight = 120;

        }

        public static final int[] kShooterSolenoidPorts = { 2, 3 };
    }

    public static final class TrenchConstants {
        public static final int kTrenchTalonPort = 15;

        public static final double kRotationSpeed = 0.6;
        public static final double kPositionSpeed = 0.25;

        public static final int kCountThreshold = 2;
        public static final double kConfidenceThreshold = 0.7;

        public static final int kMaxRotations = 4;

        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
         * or 3. Only the first two (0,1) are visible in web-based configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
         * we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        public final static double kRPM = 500.0;

        public final static double kVelocity = kRPM * 4096 / 600;

        public final static double kP = 0.25;
        public final static double kI = 0.001;
        public final static double kD = 20.0;
        public final static double kF = 1023.0 / 7200.0;
        public final static int kIz = 300;
        public final static double kPeakOut = 1.00;

        public final static boolean kSensorPhase = true;
    }

    public static final int kTimeoutMs = 10;
}
