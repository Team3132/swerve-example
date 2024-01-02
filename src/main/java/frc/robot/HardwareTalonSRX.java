package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/*
 * Package to wrap a CTRE talon SRX controller.
 * 
 * This is the hardware interface class that implements the interface we use.
 * 
 * We have a scale factor. This is useful in position and velocity close loop feedback modes.
 * For reading it we divide by the scale factor, when writing values we multiply by the scale
 * factor.
 * 
 */
public class HardwareTalonSRX {
    public enum ControlMode {
        /**
         * Set the faction of time that the motor is being powered [-1,1].
         * Same as percent output.
         */
        DutyCycle(0, com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput),
        /**
         * Position closed loop
         */
        Position(1, com.ctre.phoenix.motorcontrol.ControlMode.Position),
        /**
         * Speed closed loop
         * This is what both the Talon and the Spark MAX call velocity, but there is no
         * direction
         * component
         * so we call it speed.
         */
        Speed(2, com.ctre.phoenix.motorcontrol.ControlMode.Velocity),
        /**
         * Input current closed loop
         */
        Current(3, com.ctre.phoenix.motorcontrol.ControlMode.Current),
        /**
         * Follow other motor controller
         * Not supported by Spark MAX.
         */
        Follower(5, com.ctre.phoenix.motorcontrol.ControlMode.Follower),
        /**
         * Motion Profile
         */
        MotionProfile(6, com.ctre.phoenix.motorcontrol.ControlMode.MotionProfile),
        /**
         * Motion Magic
         */
        MotionMagic(7, com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic),
        /**
         * Motion Profile with auxiliary output
         */
        MotionProfileArc(10, com.ctre.phoenix.motorcontrol.ControlMode.MotionProfileArc),

        /**
         * Disable Motor Controller
         * Not supported by Spark MAX
         */
        Disabled(15, com.ctre.phoenix.motorcontrol.ControlMode.Disabled),

        /**
         * Voltage
         * Not natively supported by TalonSRX/Falcon, so calculated in the wrapper
         * class.
         */
        Voltage(20, com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput);

        /**
         * Value of control mode
         */
        public final int value;
        public final com.ctre.phoenix.motorcontrol.ControlMode talonControlMode;

        /**
         * Create ControlMode of initValue
         * 
         * @param initValue Value of ControlMode
         */
        ControlMode(final int initValue, com.ctre.phoenix.motorcontrol.ControlMode talonControlMode) {
            this.value = initValue;
            this.talonControlMode = talonControlMode;
        }
    };

    public class PIDF {
        public double p, i, d, f;

        public PIDF(double p, double i, double d, double f) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
        }
    }

    public class TalonSensorCollection {
        private TalonSRX talon;
        private double scale = 1;

        TalonSensorCollection(TalonSRX talon) {
            this.talon = talon;
        }

        public int getAnalogIn() {
            return talon.getSensorCollection().getAnalogIn();
        }

        public int getAnalogInRaw() {
            return talon.getSensorCollection().getAnalogInRaw();
        }

        public int getAnalogInVel() {
            return talon.getSensorCollection().getAnalogInVel();
        }

        public boolean getPinStateQuadA() {
            return talon.getSensorCollection().getPinStateQuadA();
        }

        public boolean getPinStateQuadB() {
            return talon.getSensorCollection().getPinStateQuadB();
        }

        public boolean getPinStateQuadIdx() {
            return talon.getSensorCollection().getPinStateQuadIdx();
        }

        public int getPulseWidthPosition() {
            return talon.getSensorCollection().getPulseWidthPosition();
        }

        public int getPulseWidthRiseToFallUs() {
            return talon.getSensorCollection().getPulseWidthRiseToFallUs();
        }

        public int getPulseWidthRiseToRiseUs() {
            return talon.getSensorCollection().getPulseWidthRiseToRiseUs();
        }

        public int getPulseWidthVelocity() {
            return talon.getSensorCollection().getPulseWidthVelocity();
        }

        public double getQuadraturePosition() {
            return talon.getSensorCollection().getQuadraturePosition() / scale;
        }

        public int getQuadratureVelocity() {
            return talon.getSensorCollection().getQuadratureVelocity();
        }

        public boolean isFwdLimitSwitchClosed() {
            return talon.getSensorCollection().isFwdLimitSwitchClosed();
        }

        public boolean isRevLimitSwitchClosed() {
            return talon.getSensorCollection().isRevLimitSwitchClosed();
        }

        public ErrorCode setAnalogPosition(int newPosition, int timeoutMs) {
            return talon.getSensorCollection().setAnalogPosition(newPosition, timeoutMs);
        }

        public ErrorCode setPulseWidthPosition(int newPosition, int timeoutMs) {
            return talon.getSensorCollection().setPulseWidthPosition(newPosition, timeoutMs);
        }

        public ErrorCode setQuadraturePosition(double newPosition, int timeoutMs) {
            return talon.getSensorCollection().setQuadraturePosition((int) (newPosition * scale),
                    timeoutMs);
        }

        public void setScale(double scale) {
            this.scale = scale;
        }
    }

    private TalonSensorCollection sensorCollection;
    private TalonSRX talon;
    private double scale = 1.0;
    private double lastDemand = 0;
    private ControlMode lastMode = ControlMode.Disabled;

    private boolean scalable(ControlMode mode) {
        return ((mode == ControlMode.Speed) || (mode == ControlMode.Position)
                || (mode == ControlMode.MotionMagic));
    }

    protected HardwareTalonSRX(TalonSRX talon) {
        this.talon = talon;
        sensorCollection = new TalonSensorCollection(talon);
    }

    public void set(ControlMode mode, double demand) {
        lastMode = mode;
        lastDemand = demand; // Pre-scaling.
        if (scalable(mode)) {
            demand *= scale;
        }
        if (mode.equals(ControlMode.Voltage)) {
            // TalonSRX doesn't support voltage as a control mode, so percent output is used
            // instead.
            mode = ControlMode.DutyCycle;
            demand /= talon.getBusVoltage();
        }
        if (mode.equals(ControlMode.Speed)) {
            // In Velocity mode, the talon expects position change / 100ms.
            // Convert ticks / sec to ticks / 100ms.
            demand /= 10;
        }
        talon.set(mode.talonControlMode, demand);
    }

    public void set(ControlMode mode, double demand, double arbFeedFwd) {
        lastMode = mode;
        lastDemand = demand; // Pre-scaling.
        if (scalable(mode)) {
            demand *= scale;
        }
        if (mode.equals(ControlMode.Voltage)) {
            // TalonSRX doesn't support voltage as a control mode, so percent output is used
            // instead.
            mode = ControlMode.DutyCycle;
            demand /= talon.getBusVoltage();
        }
        if (mode.equals(ControlMode.Speed)) {
            // In Velocity mode, the talon expects position change / 100ms.
            // Convert ticks / sec to ticks / 100ms.
            demand /= 10;
        }
        talon.set(mode.talonControlMode, demand, DemandType.ArbitraryFeedForward, arbFeedFwd);
    }

    public double get() {
        return lastDemand;
    }

    public HardwareTalonSRX resetEncoder(double position) {
        setSelectedSensorPosition(position, 0, 30);
        return this;
    }

    public double getPosition() {
        return talon.getSelectedSensorPosition(0) / scale;
    }

    public double getSpeed() {
        // TalonSRX::getSelectedSensorVelocity() returns ticks / 100ms.
        // Convert from ticks / 100ms to ticks / second and then scale.
        return 10 * talon.getSelectedSensorVelocity() / scale;
    }

    public HardwareTalonSRX setScale(double ticksPerTurn, double gearRatio, double unitsPerRev) {
        scale = ticksPerTurn * gearRatio / unitsPerRev;
        if (scale == 0) {
            throw new RuntimeException(
                    "WARNING: HardwareSparkMAX::setScale() was passed zero, this isn't what you want!");
        }
        return this;
    }

    public boolean isAtForwardLimit() {
        return talon.isFwdLimitSwitchClosed() == 1;
    }

    public boolean isAtReverseLimit() {
        return talon.isRevLimitSwitchClosed() == 1;
    }

    public void neutralOutput() {
        talon.neutralOutput();
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        talon.setNeutralMode(neutralMode);
    }

    public HardwareTalonSRX setSensorPhase(boolean phase) {
        talon.setSensorPhase(phase);
        return this;
    }

    public double getBusVoltage() {
        return talon.getBusVoltage();
    }

    public double getOutputPercent() {
        return talon.getMotorOutputPercent();
    }

    public double getOutputVoltage() {
        return talon.getMotorOutputVoltage();
    }

    public double getOutputCurrent() {
        return talon.getStatorCurrent();
    }

    public double getSupplyCurrent() {
        return talon.getSupplyCurrent();
    }

    public double getTemperature() {
        return talon.getTemperature();
    }

    public ErrorCode setSelectedSensorPosition(double sensorPos, int pidIdx, int timeoutMs) {
        return talon.setSelectedSensorPosition((int) (sensorPos * scale), pidIdx, timeoutMs);
    }

    public HardwareTalonSRX setInverted(boolean invert) {
        talon.setInverted(invert);
        return this;
    }

    public boolean getInverted() {
        return talon.getInverted();
    }

    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx,
            int timeoutMs) {
        return talon.configSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
    }

    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
        return talon.setStatusFramePeriod(frame, periodMs, timeoutMs);
    }

    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
        return talon.getStatusFramePeriod(frame, timeoutMs);
    }

    public ErrorCode configVelocityMeasurementPeriod(SensorVelocityMeasPeriod period,
            int timeoutMs) {
        return talon.configVelocityMeasurementPeriod(period, timeoutMs);
    }

    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs) {
        return talon.configVelocityMeasurementWindow(windowSize, timeoutMs);
    }

    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose,
            int timeoutMs) {
        return talon.configForwardLimitSwitchSource(type, normalOpenOrClose, timeoutMs);
    }

    public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose,
            int timeoutMs) {
        return talon.configReverseLimitSwitchSource(type, normalOpenOrClose, timeoutMs);
    }

    public ErrorCode configPeakCurrentLimit(int amps, int timeoutMs) {
        return talon.configPeakCurrentLimit(amps, timeoutMs);
    }

    public ErrorCode configPeakCurrentDuration(int milliseconds, int timeoutMs) {
        return talon.configPeakCurrentDuration(milliseconds, timeoutMs);
    }

    public ErrorCode configContinuousCurrentLimit(int amps, int timeoutMs) {
        return talon.configContinuousCurrentLimit(amps, timeoutMs);
    }

    public void enableCurrentLimit(boolean enable) {
        talon.enableCurrentLimit(enable);
    }

    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        return talon.configOpenloopRamp(secondsFromNeutralToFull, timeoutMs);
    }

    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        return talon.configClosedloopRamp(secondsFromNeutralToFull, timeoutMs);
    }

    public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) {
        return talon.configPeakOutputForward(percentOut, timeoutMs);
    }

    public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) {
        return talon.configPeakOutputReverse(percentOut, timeoutMs);
    }

    public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) {
        return talon.configNominalOutputForward(percentOut, timeoutMs);
    }

    public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) {
        return talon.configNominalOutputReverse(percentOut, timeoutMs);
    }

    public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) {
        return talon.configNeutralDeadband(percentDeadband, timeoutMs);
    }

    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        return talon.configVoltageCompSaturation(voltage, timeoutMs);
    }

    public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs) {
        return talon.configVoltageMeasurementFilter(filterWindowSamples, timeoutMs);
    }

    public void enableVoltageCompensation(boolean enable) {
        talon.enableVoltageCompensation(enable);
    }

    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx,
            int timeoutMs) {
        return talon.configSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
    }

    public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource,
            int remoteOrdinal,
            int timeoutMs) {
        return talon.configRemoteFeedbackFilter(deviceID, remoteSensorSource, remoteOrdinal,
                timeoutMs);
    }

    public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice,
            int timeoutMs) {
        return talon.configSensorTerm(sensorTerm, feedbackDevice, timeoutMs);
    }

    public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) {
        return talon.setControlFramePeriod(frame, periodMs);
    }

    public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs) {
        return talon.setStatusFramePeriod(frame, periodMs, timeoutMs);
    }

    public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) {
        return talon.getStatusFramePeriod(frame, timeoutMs);
    }

    public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs) {
        return talon.configForwardLimitSwitchSource(type, normalOpenOrClose, deviceID, timeoutMs);
    }

    public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type,
            LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs) {
        return talon.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID, timeoutMs);
    }

    public void overrideLimitSwitchesEnable(boolean enable) {
        talon.overrideLimitSwitchesEnable(enable);
    }

    public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs) {
        return talon.configForwardSoftLimitThreshold(forwardSensorLimit, timeoutMs);
    }

    public ErrorCode configReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs) {
        return talon.configReverseSoftLimitThreshold(reverseSensorLimit, timeoutMs);
    }

    public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) {
        return talon.configForwardSoftLimitEnable(enable, timeoutMs);
    }

    public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) {
        return talon.configReverseSoftLimitEnable(enable, timeoutMs);
    }

    public void overrideSoftLimitsEnable(boolean enable) {
        talon.overrideSoftLimitsEnable(enable);
    }

    public HardwareTalonSRX setPIDF(int slotIdx, PIDF pidf) {
        // Ignore return values.
        talon.config_kP(slotIdx, pidf.p, 10);
        talon.config_kI(slotIdx, pidf.i, 10);
        talon.config_kD(slotIdx, pidf.d, 10);
        talon.config_kF(slotIdx, pidf.f, 10);
        return this;
    }

    public HardwareTalonSRX selectProfileSlot(int slotIdx) {
        // Not implmented by default.
        return this;
    }

    public ErrorCode config_IntegralZone(int slotIdx, int izone, int timeoutMs) {
        return talon.config_IntegralZone(slotIdx, izone, timeoutMs);
    }

    public ErrorCode configAllowableClosedloopError(int slotIdx, int allowableCloseLoopError,
            int timeoutMs) {
        return talon.configAllowableClosedloopError(slotIdx, allowableCloseLoopError, timeoutMs);
    }

    public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
        return talon.configMaxIntegralAccumulator(slotIdx, iaccum, timeoutMs);
    }

    public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) {
        return talon.setIntegralAccumulator(iaccum, pidIdx, timeoutMs);
    }

    public double getClosedLoopError(int pidIdx) {
        double value = talon.getClosedLoopError(pidIdx);

        if (scalable(lastMode)) {
            value = value / scale;
        }
        return value;
    }

    public double getIntegralAccumulator(int pidIdx) {
        return talon.getIntegralAccumulator(pidIdx);
    }

    public double getErrorDerivative(int pidIdx) {
        return talon.getErrorDerivative(pidIdx);
    }

    public void selectProfileSlot(int slotIdx, int pidIdx) {
        talon.selectProfileSlot(slotIdx, pidIdx);
    }

    public double getActiveTrajectoryPosition() {
        return talon.getActiveTrajectoryPosition();
    }

    public double getActiveTrajectoryVelocity() {
        return talon.getActiveTrajectoryVelocity();
    }

    public double getActiveTrajectoryHeading() {
        // return talon.getActiveTrajectoryHeading();
        return talon.getActiveTrajectoryPosition(1);
    }

    public ErrorCode configMotionCruiseVelocity(double unitsPerSec, int timeoutMs) {
        final double sensorUnitsPerSec = unitsPerSec * scale;
        final int sensorUnitsPer100ms = (int) Math.round(sensorUnitsPerSec / 10);
        return talon.configMotionCruiseVelocity(sensorUnitsPer100ms, timeoutMs);
    }

    public ErrorCode configMotionAcceleration(double unitsPerSecPerSec, int timeoutMs) {
        final double sensorUnitsPerSecPerSec = unitsPerSecPerSec * scale;
        final int sensorUnitsPer100msPerSec = (int) Math.round(sensorUnitsPerSecPerSec / 10);
        return talon.configMotionAcceleration(sensorUnitsPer100msPerSec, timeoutMs);
    }

    public ErrorCode clearMotionProfileTrajectories() {
        return talon.clearMotionProfileTrajectories();
    }

    public int getMotionProfileTopLevelBufferCount() {
        return talon.getMotionProfileTopLevelBufferCount();
    }

    public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        return talon.pushMotionProfileTrajectory(trajPt);
    }

    public boolean isMotionProfileTopLevelBufferFull() {
        return talon.isMotionProfileTopLevelBufferFull();
    }

    public void processMotionProfileBuffer() {
        talon.processMotionProfileBuffer();
    }

    public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) {
        return talon.getMotionProfileStatus(statusToFill);
    }

    public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) {
        return talon.clearMotionProfileHasUnderrun(timeoutMs);
    }

    public ErrorCode changeMotionControlFramePeriod(int periodMs) {
        return talon.changeMotionControlFramePeriod(periodMs);
    }

    public ErrorCode getLastError() {
        return talon.getLastError();
    }

    public ErrorCode getFaults(Faults toFill) {
        return talon.getFaults(toFill);
    }

    public ErrorCode getStickyFaults(StickyFaults toFill) {
        return talon.getStickyFaults(toFill);
    }

    public ErrorCode clearStickyFaults(int timeoutMs) {
        return talon.clearStickyFaults(timeoutMs);
    }

    public int getFirmwareVersion() {
        return talon.getFirmwareVersion();
    }

    public boolean hasResetOccurred() {
        return talon.hasResetOccurred();
    }

    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
        return talon.configSetCustomParam(newValue, paramIndex, timeoutMs);
    }

    public int configGetCustomParam(int paramIndex, int timoutMs) {
        return talon.configGetCustomParam(paramIndex, timoutMs);
    }

    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal,
            int timeoutMs) {
        return talon.configSetParameter(param, value, subValue, ordinal, timeoutMs);
    }

    public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal,
            int timeoutMs) {
        return talon.configSetParameter(param, value, subValue, ordinal, timeoutMs);
    }

    public double configGetParameter(ParamEnum paramEnum, int ordinal, int timeoutMs) {
        return talon.configGetParameter(paramEnum, ordinal, timeoutMs);
    }

    public double configGetParameter(int paramEnum, int ordinal, int timeoutMs) {
        return talon.configGetParameter(paramEnum, ordinal, timeoutMs);
    }

    public int getBaseID() {
        return talon.getBaseID();
    }

    public int getDeviceID() {
        return talon.getDeviceID();
    }

    public TalonSensorCollection getSensorCollection() {
        return sensorCollection;
    }

    public HardwareTalonSRX follow(IMotorController master) {
        talon.follow(master);
        return this;
    }

    public IMotorController getHWTalon() {
        return talon;
    }

    public TalonSRX getTalonSRX() {
        return talon;
    }

    public static HardwareTalonSRX talonSRX(int canID, boolean invert, NeutralMode mode) {
        com.ctre.phoenix.motorcontrol.can.TalonSRX talon = new com.ctre.phoenix.motorcontrol.can.TalonSRX(canID);
        // Reset to factory defaults
        talon.configFactoryDefault();
        talon.setInverted(invert);
        talon.setNeutralMode(mode);
        return new HardwareTalonSRX(talon);
    }

}
