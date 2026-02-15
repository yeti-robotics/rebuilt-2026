// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int GIGA_PORT = 2;

    public static final Mode simMode = Mode.SIM;
    public static final String ALPHA_SERIAL_NUM = "032B4BBC";

    public static final Mode currentMode = System.getenv("serialnum") == "032B4BBC" ? Mode.ALPHA : System.getenv("serialnum") == "BLIZZARD_SERIAL_NUM" ? Mode.BETA : Mode.SIM;

    public static final CANBus rioBus = new CANBus("rio");

    public static enum Mode {
        /** Running on a real robot. */
        ALPHA,

        BETA,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
