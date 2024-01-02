package frc.robot;



import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.lib.ConfigReader;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.strongback.components.PIDF;

/**
 * Class responsible for updating values which are dependent on robot hardware.
 * (e.g. if subsystems are enabled or not) It reads from a text file Currently
 * the supported types are String, int, double, boolean and int array.
 * 
 * Example lines:
 * drivebase/enabled = true
 * drivebase/rampRate = 0.13125
 * drivebase/right/canIDs/withEncoders = 7,6
 * drivebase/right/canIDs/withoutEncoders = 5
 * 
 * The configuration can be overridden on each robot by changing a text file
 * stored on the robot allowing different robots to have different configuration
 * preventing having to modify the code each time it's pushed to a different bit
 * of hardware.
 * 
 * This is very useful for testing when parts of the hardware are not attached,
 * delivered or even broken.
 */
public class Config {

    
    /**
     * NavX
     * 
     * Using the gyro for autonomous routines.
     */
    public static class navx {
        public static final boolean enabled = getBoolean("navx/enabled", true);
    }

    /**
     * Location parameters
     *
     * Tracks current and historical position.
     */
    public static class location {
        public static class history {
            public static final int memorySecs = 5;
            public static final int cycleSpeedHz = 100;
        }
    }

    /**
     * PDP parameters
     * 
     * The motor controller wrappers also monitor current, so this is normally off.
     */
    public static class pdp {
        public static final boolean enabled = getBoolean("pdp/enabled", true);
        public static final int canId = getInt("pdp/canID", 0);
        // By default we do not monitor the PDP (CAN performance concerns)
        public static final boolean monitor = getBoolean("pdp/monitor", true);
        // By default we do not monitor any channels (motor controllers will also monitor)
        public static final int[] channels = getIntArray("pdp/channels", new int[0]);
    }


    /**
     * Charting important values for post match debugging.
     */
    public static class charting {
        public static final boolean enabled = getBoolean("charting/enabled", true);
    }

    /**
     * These things are immutable
     */
    public static class constants {
        public static final double fullCircle = 360.0; // size of a full circle in internal units
                                                       // (degrees)
        public static final double halfCircle = 180.0; // size of a half circle in internal units
                                                       // (degrees)
        public static final double quarterCircle = 90.0; // size of a quarter circle in internal
                                                         // units (degrees)
        public static final double inchesToMetres = 0.0254;

    }

    /**
     * Update intervals
     */
    public static class intervals {
        public static final long executorCycleMSec = 20; // 50Hz
        public static final double dashboardUpdateSec = 0.5;
    }

    /**
     * Motor controller values
     */
    public static class motorController {
        public static final String talonSRX = "TalonSRX";
        public static final String sparkMAX = "SparkMAX";
        public static final String defaultType = talonSRX;

        /**
         * Current limits
         * 
         * Normally used by motor controllers
         */
        public static class currentLimit {
            public static final int defaultContinuousAmps = 30;
            public static final int defaultPeakAmps = 40;
        }
    }

    /**
     * Encoder values
     */
    public static class encoder {
        public static final double falconTicks = 2048; // Falon inbuilt encoders.
        public static final double SparkMAXTicks = 42; // SparkMAX inbuild encoders.
        public static final double s4tTicks = 1440; // ticks per rev.
        public static final double versaIntegratedTicks = 4096; // ticks per rotation
    }

    /**
     * Server log sync parameters. Allows automatic uploading to a remote webserver.
     */
    public static class logging {
        public static final String flashDrive =
                Files.exists(Paths.get("/media/sda1")) ? "/media/sda1" : "/tmp";
        public static final String basePath = flashDrive; // log files (has to be inside web server)
        public static final String dataExtension = "data";
        public static final String dateExtension = "date";
        public static final String latestExtension = "latest";
        public static final String eventExtension = "event";

        public static class webserver {
            public static final String path = flashDrive; // where web server's data lives
            public static final int port = 5800; // port for graph/log web server
        }

        /**
         * Define parameters that govern the usage of the websocket logging server.
         */
        public static class liveloggingserver {
            public static final int port = 5803;
        }

        public static class rsync {
            // Port to forward to port 22 for transfering robot logs to pc over
            // rsync. Works around limited ports available while on the FMS.
            public static final int port = 5802;
            // Hostname of the robot.
            public static final String hostname = "roborio-3132-FRC.local";
        }
    }

    /**
     * Location on the roborio of the configuration file and config server details.
     */
    public static class config {
        public static final String homeDirectory = System.getProperty("user.home");
        public static final String configFilePath =
                Paths.get(homeDirectory, "config.txt").toString();
        public static final String robotNameFilePath =
                Paths.get(homeDirectory, "robotname.txt").toString();

        /**
         * Allow editing of config file via webserver.
         */
        public static class webserver {
            public static final String root = "/home/lvuser/deploy/www";
            public static final int port = 5801;
        }
    }

    // Only implementation from here onwards.

    private final static ConfigReader reader = new ConfigReader();

    /**
     * Needs to be called after the config is loaded to write out an example config
     * file and to print out details about the config file.
     */
    public static void finishLoadingConfig() {
        reader.finishLoadingConfig();
    }

    protected static String getMotorControllerType(final String parameterName,
            final String defaultValue) {
        return reader.getMotorControllerType(parameterName, defaultValue);
    }

    protected static int getInt(final String key, final int defaultValue) {
        return reader.getInt(key, defaultValue);
    }

    protected static double getDouble(final String key, final double defaultValue) {
        return reader.getDouble(key, defaultValue);
    }

    protected static PIDF getPIDF(final String prefix, final PIDF pidf) {
        return reader.getPIDF(prefix, pidf);
    }

    protected static boolean getBoolean(final String key, final boolean defaultValue) {
        return reader.getBoolean(key, defaultValue);
    }

    protected static String getString(final String key, final String defaultValue) {
        return reader.getString(key, defaultValue);
    }

    protected static int[] getIntArray(final String key, final int[] defaultValue) {
        return reader.getIntArray(key, defaultValue);
    }
}
