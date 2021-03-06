// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;
import java.util.ArrayList;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX motor1;
    private final WPI_TalonFX motor2;
    private final WPI_TalonFX[] motors;

    private static final double kGearRatio = 1.0;
    public static final double kLaunchAngle = 65.0;
    public static final double kFlywheelDiameter = Convert.inchesToMeters(4.0);

    private static final double kMaxFlywheelRpm = 4500;
    public static final double kMaxBallVelocity = flywheelRpmToBallVelocity(kMaxFlywheelRpm);
    private static final double kMaxAcceptableRpmError = 50.0;
    private static final int kOnTargetCounterSize = 10;

    public static final double kSpitRpm = 750.0;

    private static final double kP_Flywheel = 0.35;
    private static final double kF_Flywheel = calculateKF(2700, 0.50);

    private double targetRpm = 0.0;
    private double actualRpm = 0.0;

    private ArrayList<Boolean> onTarget = new ArrayList<>(kOnTargetCounterSize);
    // private Debouncer onTargetBool = new Debouncer(0.02*10,DebounceType.kBoth);
    // private boolean onTarget = false;

    public ShooterSubsystem() {
        motor1 = new WPI_TalonFX(Shooter.kMotor1CanId);
        motor2 = new WPI_TalonFX(Shooter.kMotor2CanId);

        motors = new WPI_TalonFX[] {motor1, motor2};

        configMotors();
    }

    public static double ballVelocityToFlywheelRpm(double ballVelocity) {
        return ballVelocity / (kFlywheelDiameter * Math.PI) * 60.0;
    }

    public static double flywheelRpmToBallVelocity(double flywheelRpm) {
        return flywheelRpm / 60.0 * (kFlywheelDiameter * Math.PI);
    }

    private void configMotors() {
        for (WPI_TalonFX motor : motors) {
            motor.configFactoryDefault();

            motor.configVoltageCompSaturation(12.0);
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

            motor.setNeutralMode(NeutralMode.Coast);

            motor.config_kF(0, kF_Flywheel);
            motor.config_kP(0, kP_Flywheel);
        }

        motor1.setInverted(true);
    }

    @Override
    public void periodic() {
        actualRpm = getRpm();
        boolean onTargetRaw = Math.abs(actualRpm - targetRpm) < kMaxAcceptableRpmError;
        // onTarget = onTargetBool.calculate(onTargetRaw);
        if (onTarget.size() > kOnTargetCounterSize) onTarget.remove(0);
        if (targetRpm > 0) {
            onTarget.add(onTargetRaw);
        } else {
            onTarget.add(false);
        }

        SmartDashboard.putNumber("SH - actual rpm", actualRpm);
        SmartDashboard.putBoolean("SH - at goal", isAtTarget());
        if (Constants.TUNING_MODE) {
            SmartDashboard.putNumber("SH - goal rpm", targetRpm);
        }
    }

    public void spit() {
        shootRpm(kSpitRpm);
    }

    public void shootRpm(double rpm) {
        targetRpm = rpm;
        for (WPI_TalonFX motor : motors) {
            motor.set(
                    ControlMode.Velocity,
                    Convert.rpmToEncoderVel(
                            clampToBounds(rpm), kGearRatio, Encoder.TalonFXIntegrated));
        }
    }

    public void shootPercent(double percent) {
        targetRpm = percent;
        for (WPI_TalonFX motor : motors) {
            motor.set(ControlMode.PercentOutput, percent);
        }
    }

    private double getRpm() {
        double sum = 0.0;
        for (WPI_TalonFX motor : motors) {
            sum +=
                    Convert.encoderVelToRpm(
                            motor.getSelectedSensorVelocity(),
                            kGearRatio,
                            Encoder.TalonFXIntegrated);
        }
        return sum / motors.length;
    }

    public void stop() {
        targetRpm = 0;
        for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0);
    }

    public boolean isAtTarget() {
        // return onTarget;
        for (Boolean b : onTarget) {
            if (!b) return false;
        }
        return true;
    }

    /**
     * To calculate the parameters for this function
     *
     * <ol>
     *   <li>Set the flywheel output to a percent output that is the average for most shots
     *   <li>Let it reach a steady state
     *   <li>Record the percent output and steady state rpm
     *   <li>Pass those values into this function to calculate the feedforward gain.
     * </ol>
     */
    private static double calculateKF(double rpmAtPercentOut, double percentOut) {
        // see the following link for an explanation of the math below
        // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#how-to-calculate-kf
        double sensorVelAtPercentOut =
                Convert.rpmToEncoderVel(rpmAtPercentOut, kGearRatio, Encoder.TalonFXIntegrated);
        return (percentOut * 1023) / sensorVelAtPercentOut;
    }

    private static double clampToBounds(double rpm) {
        if (rpm > 0.0 && rpm <= kMaxFlywheelRpm) return rpm;
        else if (rpm > kMaxFlywheelRpm) return kMaxFlywheelRpm;
        else return 0.0;
    }
}
