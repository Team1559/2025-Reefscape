package frc.lib.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.components.gyro.GyroIo;
import frc.lib.subsystems.LoggableSubsystem;
import frc.lib.subsystems.swerve.SwerveModuleIo.SwerveInputs;
import frc.lib.subsystems.vision.VisionConsumer;

public class SwerveDrive extends LoggableSubsystem implements VisionConsumer {
    private final SwerveModuleIo[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator estimator;
    private final GyroIo gyro;
    private final Translation2d[] locations;

    public SwerveDrive(String name, GyroIo gyro, SwerveModuleIo... modules) {
        super(name);
        this.gyro = gyro;
        this.modules = modules;
        Translation2d[] locations = new Translation2d[modules.length];
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < locations.length; i++) {
            locations[i] = modules[i].getLocation();
            SwerveInputs inputs = modules[i].getInputs();
            positions[i] = new SwerveModulePosition(inputs.distance, inputs.angle);
            addIo(modules[i], "Modules");
        }
        addIo(gyro, "Gyro");
        this.locations = locations;
        this.kinematics = new SwerveDriveKinematics(locations);
        this.estimator = new SwerveDrivePoseEstimator(kinematics, gyro.getInputs().yaw, positions, new Pose2d());
        // TODO: make constants
    }

    public void configureAuto(double massKg, double Moi) {
        RobotConfig config = new RobotConfig(23.2, 1682/* FIXME : probably wrong */,
                new ModuleConfig(0.0508, 6.0, 1.0, DCMotor.getKrakenX60(1), 80.0, 1), locations);

        AutoBuilder.configure(this::getPosition, this::resetPose, this::getCurrentSpeed, this::driveRobotOriented,
                new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(5)), config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);// TODO: output
    }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        driveRobotOriented(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPosition().getRotation()));
    }

    public void driveRobotOriented(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(modules[i].getInputs().angle);
            modules[i].setState(states[i]);
        }
    }

    public Pose2d getPosition() {
        return estimator.getEstimatedPosition();
    }

    public ChassisSpeeds getCurrentSpeed() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < states.length; i++) {
            SwerveInputs inputs = modules[i].getInputs();
            states[i] = new SwerveModuleState(inputs.speed, inputs.angle);
        }
        return kinematics.toChassisSpeeds(states);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < positions.length; i++) {
            SwerveInputs inputs = modules[i].getInputs();
            positions[i] = new SwerveModulePosition(inputs.distance, inputs.angle);
        }
        return positions;
    }

    public void addVisionMeasurement(Pose2d estimatedPose2d, double timestamp, Matrix<N3, N1> standardDeviation) {
        estimator.addVisionMeasurement(estimatedPose2d, timestamp, standardDeviation);
    }

    @Override
    public void periodic() {
        super.periodic();
        updateOdometry();
        log();
    }

    private void updateOdometry() {
        estimator.update(gyro.getInputs().yaw, getModulePositions());
    }

    public void resetPose(Pose2d pose) {
        estimator.resetPose(pose);
    }

    private void log() {
        Logger.recordOutput(getLogPath("EstimatedPosition"), getPosition());
        Logger.recordOutput(getLogPath("Heading"), gyro.getInputs().yaw);
        Logger.recordOutput(getLogPath("TargetVelocity"), getCurrentSpeed());
        // Logger.recordOutput(getName() + "/distanceToTag19",
        // getPosition().getTranslation().minus(new
        // Translation2d(4.074,4.745)).getNorm());
        // TODO: Just for testing :p
    }

}
