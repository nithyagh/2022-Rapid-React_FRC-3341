// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANID {
        public static final class Climber {
            public static final int testArmPivot = 0;
            public static final int testArmWinch = 1;

            public static final int LeftFrontArmPivot = 6;
            public static final int RightFrontArmPivot = 7;
            public static final int LeftRearArmPivot = 8;
            public static final int RightRearArmPivot = 9;

            public static final int LeftFrontArmWinch = 10;
            public static final int RightFrontArmWinch = 11;
            public static final int LeftRearArmWinch = 12;
            public static final int RightRearArmWinch = 13;
        }
    }

    public static final class JoystickAxis {
        public static final int XAxis = 0;
        public static final int YAxis = 1;
    }

    public static final class USBOrder {
        public static final int Zero = 0;
        public static final int One = 1;
        public static final int Two = 2;
        public static final int Three = 3;
    }

    public static final class Climber {
        public static final double MinExtendPower = 0.0;
        public static final double MaxExtendPower = 1.0;

        public static final double MinPivotPower = 0.0;
        public static final double MaxPivotPower = 0.4;
    }

    public static final class AHRS {
        public static final double MinDeadband = -5;
        public static final double MaxDeadband = 5;
    }

    public static final class AnalogPort {
        public static final int Zero = 0;
        public static final int One = 1;
        public static final int Two = 2;
        public static final int Three = 3;
    }
}
