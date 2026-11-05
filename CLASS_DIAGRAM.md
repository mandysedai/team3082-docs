# 2026Rebuilt class diagrams

The project has 55 types (classes, enums, an interface and an annotation), so the
diagrams are split by area. Most subsystems are all-static "manager" classes with
`init()` / `update()` methods instead of WPILib `Subsystem` objects, so most of
the arrows are «use» dependencies, not fields. Static members end with `$`.

1. [Overview](#1-overview)
2. [Swerve drive and localization](#2-swerve-drive-and-localization)
3. [Scoring subsystems](#3-scoring-subsystems)
4. [Autonomous and trajectories](#4-autonomous-and-trajectories)
5. [Utilities and configuration](#5-utilities-and-configuration)

## 1. Overview

`Robot` drives the loop: each periodic call runs `update()` on the managers below.

```mermaid
classDiagram
    direction LR
    class LoggedRobot
    <<AdvantageKit>> LoggedRobot
    class Main
    <<final>> Main
    class Robot
    class OI
    <<static>> OI
    class Telemetry
    <<static>> Telemetry
    class Auto
    class FeatherFlow
    <<static>> FeatherFlow
    class SwerveManager
    <<static>> SwerveManager
    class SwervePosition
    <<static>> SwervePosition
    class SwervePID
    <<static>> SwervePID
    class Odometry
    <<static>> Odometry
    class Pigeon
    <<static>> Pigeon
    class VisionManager
    <<static>> VisionManager
    class ShooterManager
    <<static>> ShooterManager
    class Shooter
    <<static>> Shooter
    class Intake
    <<static>> Intake
    class Indexer
    <<static>> Indexer
    class LEDManager
    <<static>> LEDManager

    LoggedRobot <|-- Robot
    Main ..> Robot : «create»
    Robot ..> OI
    Robot ..> Telemetry
    Robot ..> Auto
    Robot ..> FeatherFlow
    Robot ..> SwerveManager
    Robot ..> SwervePosition
    Robot ..> Pigeon
    Robot ..> ShooterManager
    Robot ..> Intake
    Robot ..> Indexer
    Robot ..> LEDManager
    OI ..> SwerveManager
    OI ..> SwervePID
    OI ..> ShooterManager
    ShooterManager ..> Shooter
    SwervePosition ..> Odometry
    Odometry ..> VisionManager
    Odometry ..> Pigeon
```

## 2. Swerve drive and localization

```mermaid
classDiagram
    direction TB
    class SwerveManager {
        <<static>>
        +SwerveModule[] mods$
        +double rotationSpeed$
        +Vector2 movement$
        +init()$
        +update()$
        +resetSensors()$
        +rotateAndDrive(double rotSpeed, Vector2 move)$
        +rotateAndDriveFF(double rotSpeed, Vector2 move)$
        +plant()$
        +pointWheels(double radians)$
        +getDriveVelocity(int id)$ double
        +getSteerAngle(int id)$ double
        +getRobotDriveVelocity()$ Vector2
        +getRotationalVelocity()$ double
        +returnStates()$ SwerveModuleState[]
    }
    class SwerveModule {
        +TalonFX steer
        +TalonFX drive
        +CANcoder absEncoder
        +Vector2 pos
        +double targetAngle
        +double targetSpeed
        +boolean inverted
        +SwerveModuleSim simModule
        +SwerveModule(int steerID, int driveID, double cancoderOffset, double x, double y)
        +update()
        +resetSteerSensor()
        +drive(double power)
        +rotateToRad(double angle)
        +getSteerAngle() double
        +getDriveVelocity() double
        +getDrivePosition() double
    }
    class SwerveModuleSim {
        +double angle
        +double speed
        -PIDController anglePID
        +update(boolean inverted)
        +setAngle(double setPos)
        +setSpeed(double setSpeed)
        +getAngle() double
        +getSpeed() double
        +getDrivePosition() double
    }
    class Odometry {
        <<static>>
        -Vector2 position$
        -OdometryBuffer odometryBuffer$
        -Thread odomThread$
        +init()$
        +getPosition()$ Vector2
        +getVelocity()$ Vector2
        +setPosition(Vector2 pos)$
        +poseExponentiation(Vector2 deltaPos, double theta0, double deltaTheta)$ Vector2
    }
    class OdometryBuffer {
        -Vector2[] buffer
        -Vector2[] velBuffer
        +addValue(Vector2 newValue)
        +getTotalBuffer() Vector2
        +getVelocity() Vector2
    }
    class SwervePosition {
        <<static>>
        +init()$
        +update()$
        +getPosition()$ Vector2
        +getAbsVelocity()$ Vector2
        +setPosition(Vector2 newPosition)$
        +getPositionPolar()$ double[]
        +getAngleOffsetToTarget(Vector2 desired)$ double
        +getPose()$ Pose2d
    }
    class SwervePID {
        <<static>>
        +PIDController movePID$
        +PIDController rotPID$
        +init()$
        +setDestState(Vector2 dest, double destRot)$
        +updateOutputVel()$ Vector2
        +updateOutputRot()$ double
        +atDest()$ boolean
        +atRot()$ boolean
        +getError()$ Vector2
        +getRotationError()$ double
    }
    class Pigeon {
        <<static>>
        +Pigeon2 pigeon$
        +init()$
        +update()$
        +reset()$
        +setYaw(double deg)$
        +setYawRad(double rad)$
        +getRotationRad()$ double
        +getPitchRad()$ double
        +getRollRad()$ double
        +getDeltaRotRad()$ double
    }
    class VisionManager {
        <<static>>
        -Camera[] cameras$
        -boolean enabled$
        +init()$
        +getPosition(double pigeonAngle)$ Optional~Vector2~
        +getRotation(double pigeonAngle)$ Optional~Double~
        +enableVision()$
        +disableVision()$
        +isEnabled()$ boolean
        +enableLeftCam()$
        +disableLeftCam()$
        +enableRightCam()$
        +disableRightCam()$
    }
    class Camera {
        +PhotonCamera photonCamera
        +Vector2 robotToCamera
        +double cameraPitch
        +double cameraRoll
        +double cameraYaw
        +PhotonTrackedTarget latestTarget
        +isLatestTarget(PhotonTrackedTarget t) boolean
        +setLatestTarget(PhotonTrackedTarget t)
        +enable()
        +disable()
        +isDisabled() boolean
    }
    class AprilTag {
        -Vector3 pos
        -double rot
        -int id
        +getPosition() Vector2
        +get3DPosition() Vector3
        +getRotation() double
        +getID() int
    }
    class SwerveBaseVisualizer {
        <<static>>
        +init()$
        +update()$
    }
    class PIDController
    class RotationalPIDController

    SwerveManager "1" *-- "4" SwerveModule : mods
    SwerveModule *-- SwerveModuleSim : simModule
    SwerveModuleSim *-- PIDController : anglePID
    Odometry *-- OdometryBuffer
    Odometry ..> SwerveManager
    Odometry ..> Pigeon
    Odometry ..> VisionManager
    SwervePosition ..> Odometry
    SwervePosition ..> Pigeon
    SwervePID o-- PIDController : movePID
    SwervePID o-- RotationalPIDController : rotPID
    PIDController <|-- RotationalPIDController
    SwervePID ..> SwervePosition
    SwerveManager ..> Pigeon
    VisionManager "1" *-- "2" Camera
    VisionManager ..> AprilTag : Constants.APRIL_TAGS
    SwerveBaseVisualizer ..> SwerveManager
```

## 3. Scoring subsystems

```mermaid
classDiagram
    direction TB
    class ShooterManager {
        <<static>>
        +ShooterState shooterState$
        -ShooterTarget target$
        +boolean inRange$
        -ShooterManagerState targetingState$
        +init()$
        +update()$
        +shoot()$
        +stopShooting()$
        +zeroHood()$
        +setNormalAiming()$
        +setManualTower()$
        +setManualTrench()$
        +setTarget(ShooterTarget t)$
        +getTarget()$ ShooterTarget
        +getShooterState()$ ShooterState
        +getTargetingState()$ ShooterManagerState
        +predictRobotPos()$ Vector2
    }
    class ShooterManagerState {
        <<enumeration>>
        NORMAL
        MANUAL_TOWER
        MANUAL_TRENCH
    }
    class Shooter {
        <<static>>
        -TalonFX flywheelMotor$
        -TalonFX hoodMotor$
        +init()$
        +update()$
        +setTargetAngle(double angle)$
        +setTargetSpeed(double speed)$
        +getTargetAngle()$ double
        +getTargetSpeed()$ double
        +getAngle()$ double
        +getVelocity()$ double
        +atAngle()$ boolean
        +atRampedSpeed()$ boolean
    }
    class AutoTarget {
        <<static>>
        +init()$
        +getTarget()$ ShooterTarget
        +nearTrench()$ boolean
    }
    class ShooterState {
        <<enumeration>>
        IDLE
        REVVING
        SHOOTING
        ZEROING
    }
    class ShooterTarget {
        <<enumeration>>
        HUB
        PASS_LEFT
        PASS_RIGHT
        +Vector2 pos
    }
    class ShooterTableValue {
        +ShooterTableValue(double distance, double angle, double speed)
        +getDist() double
        +getAngle() double
        +getSpeed() double
    }
    class Intake {
        <<static>>
        -TalonFX pivotMotor$
        -TalonFX rollerMotor$
        +IntakeState rollerState$
        +init()$
        +update()$
        +startIntaking()$
        +stopIntaking()$
        +reverse()$
        +startFeeding(double percent)$
        +retract()$
        +getIntakeState()$ IntakeState
        +getAngle()$ double
        +getTargetAngle()$ double
        +getSpeed()$ double
    }
    class IntakeState {
        <<enumeration>>
        RESTING
        INTAKING
        REVERSE
        FEEDING
        IN_ROBOT
    }
    class Indexer {
        <<static>>
        -TalonFX spindexerMotor$
        -TalonFX handoffMotor$
        -IndexerState indexerState$
        +init()$
        +update()$
        +reverse()$
        +setNormalMode()$
        +getIndexerState()$ IndexerState
        +getSpindexerSpeed()$ double
        +getHandoffSpeed()$ double
    }
    class IndexerState {
        <<enumeration>>
        NORMAL
        REVERSE
    }
    class LEDManager {
        <<static>>
        +AddressableLED m_led$
        +AddressableLEDBuffer m_ledBuffer$
        +LEDPattern currentPattern$
        +init()$
        +update()$
        +setColor(Colors color)$
    }
    class Colors {
        <<enumeration>>
        RED
        GREEN
        BLUE
        YELLOW
        YELLOW_GREEN
    }
    class LEDMech2D {
        <<static>>
        +init()$
        +update()$
    }
    class SubsystemVisualizer {
        <<static>>
        -WheelMech flywheel$
        -WheelMech intakeWheel$
        +init()$
        +update()$
    }
    class WheelMech {
        -double currentAngle
        -int numSpokes
        +WheelMech(String name, MechanismObject2d base, int numSpokes, double radius, double width, Color8Bit color)
        +update(double velocity)
    }

    ShooterManager --> ShooterState
    ShooterManager --> ShooterTarget
    ShooterManager *-- ShooterManagerState : «inner»
    ShooterManager ..> Shooter
    ShooterManager ..> ShooterTableValue : Tuning.Shooter tables
    Shooter ..> AutoTarget
    AutoTarget ..> ShooterTarget
    Indexer ..> ShooterManager
    Indexer *-- IndexerState : «inner»
    Intake *-- IntakeState : «inner»
    LEDManager *-- Colors : «inner»
    LEDManager ..> ShooterManager
    LEDManager ..> Intake
    LEDMech2D ..> LEDManager
    SubsystemVisualizer "1" *-- "2" WheelMech
    SubsystemVisualizer ..> Shooter
    SubsystemVisualizer ..> Intake
    SubsystemVisualizer ..> Indexer
```

## 4. Autonomous and trajectories

```mermaid
classDiagram
    direction TB
    class Command
    <<WPILib>> Command
    class InstantCommand
    <<WPILib>> InstantCommand
    class Auto {
        +RoutineManager routineManager$
        +Left3Piece() SequentialCommandGroup
        +Right3Piece() SequentialCommandGroup
        +ShootNoMove() SequentialCommandGroup
        +OutpostFromCenter() SequentialCommandGroup
        +DepotToCenter() SequentialCommandGroup
        +CenterToDepot() SequentialCommandGroup
        +init()$
        +startRoutine()$
        +update()$
        +disable()$
        +getAutoSelector()$ SendableChooser~String~
    }
    class AutoRoutine
    <<annotation>> AutoRoutine
    class RoutineManager {
        -HashMap~String, CommandRoutine~ routineMap
        +SendableChooser~String~ autoSelector
        +addRoutine(String name, CommandSupplier initMethod)
        +addClass(T commandClass)
        +getCurrentCommand() SequentialCommandGroup
        +getAutoSelector() SendableChooser~String~
    }
    class CommandRoutine {
        -String sendableName
        -CommandSupplier initMethod
        +getCommand() SequentialCommandGroup
        +getName() String
    }
    class CommandSupplier {
        <<interface>>
        +getCommands() SequentialCommandGroup
    }
    class FollowPath {
        -HolonomicDriveController holonomicDriveController
        +FollowPath(ProfiledPath path, FeatherEvent[] events)
        +initialize()
        +execute()
        +end(boolean interrupted)
        +isFinished() boolean
    }
    class HolonomicDriveController {
        -PIDController xPositionPID
        -PIDController yPositionPID
        -PIDController rotationPID
        -ProfiledPath path
        +initialize(ProfiledPath path)
        +calculate() Vector2
        +calculateRotation() double
        +isFinished() boolean
        +getPositionError() Vector2
    }
    class RotateAndDriveTo {
        -Vector2 pos
        -double rot
        +RotateAndDriveTo(double angle, Vector2 position)
    }
    class Shoot
    class StartIntake
    class TuneSpeed {
        +TuneSpeed(double motorPercentOutput, double time)
    }
    class FeatherFlow {
        <<static>>
        +init()$
        +getPath(String pathName, boolean flipped)$ FeatherPath
        +buildFeatherAuto(String pathName, Command... commands)$ SequentialCommandGroup
        +buildFeatherAuto(String pathName, boolean flipped, Command... commands)$ SequentialCommandGroup
    }
    class FeatherPath {
        +List~ProfiledPath~ paths
        +List~FeatherActionDescriptor~ actions
    }
    class FeatherActionDescriptor {
        +double t
        +String type
        +double duration
        +double heading
        +boolean stopping
    }
    class FeatherEvent {
        +double t
        +Command command
    }
    class ProfiledPath {
        -ArrayList~ProfiledPoint~ profiledPoints
        +getPointAtTime(double t) ProfiledPoint
        +generateSimplifiedProfile(...)$ ProfiledPath
        +getStartPoint() Vector2
        +getStartHeading() double
        +getDuration() double
    }
    class ProfiledPoint {
        +getPosition() Vector2
        +getVelocity() Vector2
        +getCurvature() double
        +getAcceleration() double
        +getTime() double
        +getDistance() double
        +getHeading() double
        +getRotationalVelocity() double
    }
    class RobotPath {
        -List~Vector2~ points
        -List~Double~ curvatures
        -double totalLength
        +getPointAt(double t) Vector2
        +getLengthAt(double t) double
        +getSubPath(double startT, double endT) RobotPath
        +split(List~Double~ tValues) List~RobotPath~
        +getTotalLength() double
    }
    class CubicBezierCurve {
        -Vector2 p0, p1, p2, p3
        +getPoints() Vector2[]
        +getCurvatures() double[]
        +getPoint(double t) Vector2
        +getCurvatureAt(double t) double
    }

    Command <|-- FollowPath
    Command <|-- RotateAndDriveTo
    Command <|-- Shoot
    Command <|-- TuneSpeed
    Command <|-- InstantCommand
    InstantCommand <|-- StartIntake
    Auto *-- RoutineManager
    Auto ..> AutoRoutine : annotates routines
    Auto ..> FeatherFlow
    Auto ..> Shoot
    Auto ..> StartIntake
    RoutineManager "1" *-- "*" CommandRoutine
    RoutineManager ..> AutoRoutine : reflection
    CommandRoutine --> CommandSupplier
    FeatherFlow ..> FeatherPath : loads .ff files
    FeatherFlow ..> CubicBezierCurve
    FeatherFlow ..> RobotPath
    FeatherFlow ..> FollowPath : «create»
    FeatherFlow ..> RotateAndDriveTo : «create»
    FeatherFlow ..> FeatherEvent : «create»
    FeatherPath "1" *-- "*" ProfiledPath
    FeatherPath "1" *-- "*" FeatherActionDescriptor
    FollowPath *-- HolonomicDriveController
    FollowPath o-- "*" FeatherEvent
    HolonomicDriveController --> ProfiledPath
    ProfiledPath "1" *-- "*" ProfiledPoint
    ProfiledPath ..> RobotPath : built from
    FeatherEvent --> Command
```

`HolonomicDriveController` uses WPILib's `edu.wpi.first.math.controller.PIDController`,
not the project's own `frc.robot.utils.PIDController`.

## 5. Utilities and configuration

```mermaid
classDiagram
    direction TB
    class Sendable
    <<interface>> Sendable
    class PIDController {
        -int instances$
        #double dest
        +double kP, kI, kD
        +double deadband
        +double velDeadband
        +double max
        +PIDController(double p, double i, double d, double deadband, double velDeadband, double max)
        +setDest(double dest)
        +getDest() double
        +updateOutput(double pos) double
        +atSetpoint() boolean
        +isActive() boolean
        +getError() double
        +initSendable(SendableBuilder builder)
    }
    class RotationalPIDController {
        +setDest(double dest)
        +updateOutput(double pos) double
    }
    class TrapezoidalProfile {
        -double maxVelocity
        -double maxAcceleration
        -double maxDeceleration
        +TrapezoidalProfile(TrapezoidalPreset preset)
        +generateProfile(double totalDistance)
        +getVelocity(double distance) double
        +printProfile()
    }
    class TrapezoidalPreset {
        <<enumeration>>
        MOVE_PRECISE
        MOVE_FAST
        ROT_PRECISE
    }
    class Vector2 {
        +double x
        +double y
        +fromPolar(double theta, double r)$ Vector2
        +add(Vector2 v) Vector2
        +sub(Vector2 v) Vector2
        +mul(double s) Vector2
        +div(double s) Vector2
        +mag() double
        +norm() Vector2
        +rotate(double angle) Vector2
        +dot(Vector2 v) double
        +atan2() double
        +dist(Vector2 p) double
        +copy() Vector2
        +convertToFieldCoords() Vector2
    }
    class Vector3 {
        +add(Vector3 u, Vector3 v)$ Vector3
        +sub(Vector3 u, Vector3 v)$ Vector3
        +cross(Vector3 u, Vector3 v)$ Vector3
        +dot(Vector3 u, Vector3 v)$ double
        +scale(double s) Vector3
        +mag() double
        +unitVector() Vector3
        +getX() double
        +getY() double
        +getZ() double
        +getPitch() double
    }
    class RMath {
        <<static>>
        +targetAngleAbsolute(double cur, double target, double range)$ double
        +smoothJoystick1(double val)$ double
        +smoothJoystick2(Vector2 dir)$ Vector2
        +clamp(double in, double min, double max)$ double
        +interpolate(double a, double b, double t)$ double
        +interpolateRotationRad(double a, double b, double t)$ double
        +deadband(double value, double center, double deadband)$ double
        +modulo(double a, double b)$ double
    }
    class RTime {
        <<static>>
        +init()$
        +update()$
        +updateAbsolute()$
        +now()$ double
        +deltaTime()$ double
        +absolute_frame()$ long
        +createTimestamp()$ String
    }
    class Constants {
        <<static>>
        +AprilTag[] APRIL_TAGS$
    }
    class Constants_Swerve["Constants.Swerve"]
    class Constants_Shooter["Constants.Shooter"]
    class Constants_Intake["Constants.Intake"]
    class Constants_Indexer["Constants.Indexer"]
    class Constants_Traj["Constants.TrajectoryConstants"]
    class Tuning
    <<static>> Tuning
    class Tuning_OI["Tuning.OI"]
    class Tuning_Shooter["Tuning.Shooter"] {
        +ShooterTableValue[] SHOOTER_TABLE_HUB$
    }
    class Tuning_Indexer["Tuning.Indexer"]
    class Tuning_Intake["Tuning.Intake"]
    class LogitechF310 {
        <<static>>
        +int AXIS_LEFT_X$
        +int AXIS_LEFT_Y$
        +int AXIS_RIGHT_X$
        +int BUTTON_A$
        ...
    }
    class OI {
        <<static>>
        -Joystick driverStick$
        -Joystick operatorStick$
        +boolean manualAim$
        +init()$
        +update()$
    }
    class Telemetry {
        <<static>>
        +Mechanism2d subsystemViewSide$
        +Mechanism2d subsystemViewTop$
        +Mechanism2d swerveView$
        +init()$
        +update()$
    }

    Sendable <|.. PIDController
    PIDController <|-- RotationalPIDController
    TrapezoidalProfile *-- TrapezoidalPreset : «inner»
    RMath ..> Vector2
    Constants *-- Constants_Swerve : «inner»
    Constants *-- Constants_Shooter : «inner»
    Constants *-- Constants_Intake : «inner»
    Constants *-- Constants_Indexer : «inner»
    Constants *-- Constants_Traj : «inner»
    Tuning *-- Tuning_OI : «inner»
    Tuning *-- Tuning_Shooter : «inner»
    Tuning *-- Tuning_Indexer : «inner»
    Tuning *-- Tuning_Intake : «inner»
    OI ..> LogitechF310
```

`Vector2`, `RMath`, `RTime`, `Constants` and `Tuning` are used almost everywhere, so
the other diagrams leave those arrows out.
