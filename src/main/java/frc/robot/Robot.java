// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.net.InetSocketAddress;
import java.util.Calendar;

import org.jibble.simplewebserver.SimpleWebServer;
import org.strongback.Executable;
import org.strongback.Strongback;
import org.strongback.Executor.Priority;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.ConfigServer;
import frc.robot.lib.LogServer;
import frc.robot.lib.chart.Chart;
import frc.robot.lib.log.Log;
import frc.robot.lib.log.LogFileNumber;
import frc.robot.lib.log.LogHelper;

public class Robot extends TimedRobot implements Executable, LogHelper {
    //private final Controls m_controller = new Xbox();
    private final Controls m_controller = new Flightsticks();

    private Drivetrain m_swerve;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private LogServer logServer;

    @Override
    public void robotInit() {
        startWebServer();
        startConfigServer();
        info("Waiting for driver's station to connect before setting up UI");
        // Do the reset of the initialization in init().
    }

    private boolean setupCompleted = false;

    public void maybeInit() {
        if (setupCompleted)
            return;
        try {
            init();
            setupCompleted = true;
        } catch (Exception e) {
            // Write the exception to the log file.
            exception("Exception caught while initializing robot", e);
            throw e; // Cause it to abort the robot startup.
        }
    }

    /**
     * Initialize the robot now that the drivers station has connected.
     */
    public void init() {
        warning("Initialization started");
        Strongback.logConfiguration();
        Strongback.setExecutionPeriod(Config.intervals.executorCycleMSec);
        startLogServer();
        warning("Log server started");

        //createInputDevices();
        Chart.register(() -> getXSpeed(), "joystick/speed/x");
        Chart.register(() -> getYSpeed(), "joystick/speed/y");
        Chart.register(() -> getRotation(), "joystick/rotate");

        // Setup the hardware/subsystems. Listed here so can be quickly jumped to.
        m_swerve = new Drivetrain();

        warning("Subsystems created");

        createTimeEventSymlinks(); // All subsystems have registered by now, enable logging.
        if (!Config.charting.enabled) {
            error("Chart sampling disabled");
        } else {
            // Low priority means run every 20 * 4 = 80ms, or at 12.5Hz
            // It polls almost everything on the CAN bus, so don't want it to be too fast.
            Strongback.executor().register(new Chart(), Priority.LOW);
        }
        Strongback.executor().register(this, Priority.LOW);
        warning("Event symlinks created");

        // Start the scheduler to keep all the subsystems working in the background.
        Strongback.start();
        warning("Strongback started");

        // Write out the example config and print any config warnings.
        Config.finishLoadingConfig();
        warning("Config loaded");

        warning("Robot initialization successful");
    }

        /**
     * Called when the robot starts the disabled mode. Normally on first start and
     * after teleop and autonomous finish.
     */
    @Override
    public void disabledInit() {
        warning("disabledInit called");
        // Start forwarding to port 22 (ssh port) for pulling logs using rsync.
        PortForwarder.add(Config.logging.rsync.port, Config.logging.rsync.hostname, 22);
        maybeInit(); // Called before robotPeriodic().

        // Tell the controller to give up on whatever it was processing.
        //controller.disable();
        // Disable all subsystems
        //subsystems.disable();
        warning("disabledInit complete");
    }

    @Override
    public void robotPeriodic() {
        m_swerve.updateShuffleboard();
        m_swerve.updateOdometry();
    }

        /**
     * Called once when the autonomous period starts.
     */
    @Override
    public void autonomousInit() {
        warning("autonomousInit called");
        maybeInit();
        PortForwarder.remove(Config.logging.rsync.port); // Stop forwarding port to stop rsync and
                                                         // save bandwidth.
        Log.restartLogs();
        Chart.restartCharts();
        createTimeEventSymlinks();
        info("auto has started");
        //controller.enable();
        //subsystems.enable();
        warning("Controller and subsystems enabled for auto");

        //controller.run(Routines.getStartRoutine());
        //Pose2d resetPose = new Pose2d(0, 0, new Rotation2d(0));
        //subsystems.location.setCurrentPose(resetPose);

        // Kick off the selected auto program.
        //auto.executedSelectedRoutine(controller);
        //warning("Auto routine started");
    }

    @Override
    public void autonomousPeriodic() {
        driveWithJoystick(false);
    }

    /**
     * Called once when the teleop period starts.
     */
    @Override
    public void teleopInit() {
        warning("teleopInit called");
        maybeInit();
        // Stop forwarding port to stop rsync and save bandwidth.
        PortForwarder.remove(Config.logging.rsync.port);
        Log.restartLogs();
        Chart.restartCharts();
        createTimeEventSymlinks();
        //controller.enable();
        //subsystems.enable();
        //warning("Controller and subsystems enabled for teleop");
        //controller.run(Routines.setDrivebaseToDefault());
        //subsystems.ledStrip.setAlliance(getAllianceLEDColour().get());
        warning("teleopInit complete");
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
    }

    private double getXSpeed() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        return -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getXSpeed(), 0.1))
                * Drivetrain.kMaxSpeed;
    }

    private double getYSpeed() {
        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        return -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getYSpeed(), 0.1))
                * Drivetrain.kMaxSpeed;
    }

    private double getRotation() {
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        return -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getTurn(), 0.05))
                * Drivetrain.kMaxAngularSpeed;
    }

    private void driveWithJoystick(boolean fieldRelative) {

        final var xSpeed = getXSpeed();
        final var ySpeed = getYSpeed();
        final var rot = getRotation();

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rot", rot);

        m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    /**
     * Start a websocket to publish new log messages to websocket clients
     */
    private void startLogServer() {
        logServer =
                new LogServer(
                        new InetSocketAddress("0.0.0.0", Config.logging.liveloggingserver.port));
        logServer.setReuseAddr(true);
    }

    /**
     * Create the simple web server so we can interrogate the robot during
     * operation. The web server lives on a port that is available over the
     * firewalled link. We use port 5800, the first of the opened ports.
     * 
     */
    private void startWebServer() {
        File fileDir = new File(Config.logging.webserver.path);
        try {
            new SimpleWebServer(fileDir, Config.logging.webserver.port);
            debug("WebServer started at port: " + Config.logging.webserver.port);
        } catch (Exception e) {
            debug("Failed to start webserver on directory " + fileDir.getAbsolutePath());

            e.printStackTrace();
        }
    }

    /**
     * Creates the web server for allowing easy modification of the robot's config
     * file using port 5801.
     */
    private void startConfigServer() {
        String webRoot = Robot.isReal() ? Config.config.webserver.root : "src/main/deploy/www";
        try {
            new ConfigServer(webRoot, Config.config.configFilePath,
                    Config.config.robotNameFilePath, Config.config.webserver.port);
            debug("Config webserver started at port: " + Config.config.webserver.port);
        } catch (Exception e) {
            debug("Failed to start config webserver.");
            e.printStackTrace();
        }
    }

        /**
     * Create date and fms log symbolic links
     */
    private void createTimeEventSymlinks() {
        // Tell the logger what symbolic link to the log file based on the match name to
        // use.
        String matchDescription = "NoMatchDesc";
        if (DriverStation.getMatchType().toString() != "None") {
            matchDescription =
                    String.format("%s_%s_M%d_R%d_%s_P%d_L%d", DriverStation.getEventName(),
                            DriverStation.getMatchType().toString(), DriverStation.getMatchNumber(),
                            DriverStation.getReplayNumber(), DriverStation.getAlliance().toString(),
                            DriverStation.getLocation(), LogFileNumber.get());
        }
        Chart.registrationComplete(matchDescription);
        Log.createDateFiles(Calendar.getInstance(), matchDescription);
    }

    @Override
    public void execute(long timeInMillis) {
        // Logger.debug("Updating smartDashboard");
        maybeUpdateSmartDashboard();
    }

    private double lastDashboardUpdateSec = 0;

    /**
     * Possibly update the smartdashboard. Don't do this too often due to the amount
     * that is sent to the dashboard.
     */
    private void maybeUpdateSmartDashboard() {
        double now = Strongback.timeSystem().currentTime();
        if (now < lastDashboardUpdateSec + Config.intervals.dashboardUpdateSec)
            return;
        lastDashboardUpdateSec = now;
        //subsystems.updateDashboard();
        // pdp.updateDashboard();
        //controller.updateDashboard();
    }

    @Override
    public String getName() {
        return "Robot";
    }
}
