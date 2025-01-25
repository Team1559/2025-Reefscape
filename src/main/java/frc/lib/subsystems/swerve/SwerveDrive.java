package frc.lib.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModuleIo[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator estimator;
    private final Supplier<Rotation2d> heading;
    private final Translation2d[] locations;

    public SwerveDrive(Supplier<Rotation2d> heading, SwerveModuleIo... modules) {
        this.heading = heading;
        this.modules = modules;
        Translation2d[] locations = new Translation2d[modules.length];
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < locations.length; i++) {
            locations[i] = modules[i].getLocation();
            positions[i] = modules[i].getPosition();
        }
        this.locations = locations;
        this.kinematics = new SwerveDriveKinematics(locations);
        this.odometry = new SwerveDriveOdometry(kinematics, heading.get(), positions);
        this.estimator = new SwerveDrivePoseEstimator(kinematics, heading.get(), positions, new Pose2d());
        // TODO: make constants

    }

    public void configureAuto(double massKg, double Moi) {
        RobotConfig config = new RobotConfig(23.2, 1682/* FIXME : probably wrong */,
                new ModuleConfig(0.0508, 6.0, 1.0, DCMotor.getKrakenX60(1), 80.0, 1), locations);

        AutoBuilder.configure(this::getPosition, this::resetPose, this::getChassisSpeeds, this::driveRobotOriented,
                new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(5)), config, () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);// TODO: output
    }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        driveRobotOriented(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPosition().getRotation()));
    }

    public void driveRobotOriented(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(modules[i].getAngle());
            modules[i].setState(states[i]);
        }
    }

    public Pose2d getPosition() {
        return estimator.getEstimatedPosition();
    }

    public ChassisSpeeds getChassisSpeeds() {

        SwerveModuleState[] speeds = new SwerveModuleState[modules.length];
        for (int i = 0; i < speeds.length; i++) {
            speeds[i] = modules[i].getCurrentState();
        }
        return kinematics.toChassisSpeeds(speeds);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public void addVisionMeasurement(Pose2d estimatedPose2d, double timestamp, Matrix<N3, N1> standardDeviation) {
        estimator.addVisionMeasurement(estimatedPose2d, timestamp, standardDeviation);
    }

    @Override
    public void periodic() {
        log();
        updateOdometry();
    }

    private void updateOdometry() {
        odometry.update(heading.get(), getModulePositions());
    }

    private String moduleLogPrefix(int moduleIndex) {
        return getName() + "/modules/" + modules[moduleIndex].getName();
    }

    public void resetPose(Pose2d pose) {
        estimator.resetPose(pose);
    }

    private void log() {
        for (int i = 0; i < modules.length; i++) {
            Logger.recordOutput(moduleLogPrefix(i) + "/currentState/angle", modules[i].getAngle());
            Logger.recordOutput(moduleLogPrefix(i) + "/currentState/speed", modules[i].getSpeed());
            Logger.recordOutput(moduleLogPrefix(i) + "/setpoint/angle", modules[i].getSetpoint().angle);
            Logger.recordOutput(moduleLogPrefix(i) + "/setpoint/speed", modules[i].getSetpoint().speedMetersPerSecond);
            Logger.recordOutput(moduleLogPrefix(i) + "/driveMotor/current", modules[i].getDriveMotorCurrent());
            Logger.recordOutput(moduleLogPrefix(i) + "/driveMotor/temperature", modules[i].getDriveMotorTemperature());
            Logger.recordOutput(moduleLogPrefix(i) + "/steerMotor/current", modules[i].getSteerMotorCurrent());
            Logger.recordOutput(moduleLogPrefix(i) + "/steerMotor/temperature", modules[i].getSteerMotorTemperature());
        }

        Logger.recordOutput(getName() + "/estimatedPosition", getPosition());
        Logger.recordOutput(getName() + "/heading", heading.get());
    }

}
