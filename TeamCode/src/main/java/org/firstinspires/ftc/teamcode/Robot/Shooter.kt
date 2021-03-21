package org.firstinspires.ftc.teamcode.Robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.util.InterpLUT
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.Util.HardwareNames
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt


/*
Control of the shooter's vertical angle is either manual or automatic.

Manual control will simply not apply any automatic control, and the servo must be given a position
to go to.

Automatic control will use a look up table to calculate the angle needed to go to a
position.

The position to aim towards can be set with "shooterTarget". Positions for the high goals
and powershots are stored in the enum "AllianceTarget".

The flywheel uses a feedforward controller to keep a constant velocity. It can be toggled on and off.
 */
@Config
class Shooter(robot: Robot): Component(robot) {


    //Control schemes of the shooter's vertical angle
    enum class AngleMode {
        MANUAL,
        AUTO
    }


    companion object{
        @JvmField
        var flywheelVeloCoefficients = PIDCoefficients(kP = 2.0)

        @JvmField
        var flywheelVelocitykV: Double = 12.0;

        //The target velocity for the flywheel when it is on. In terms of ticks per rev of the encoder
        private val flywheelTargetVelocity = 100000.0; //TODO: Find this


        //Returns angle to a target in radians from the x-axis
        fun angleToTarget(current: Pose2d, target: Pose2d): Double{
            var a: Double = atan2(target.y - current.y, target.x - current.x)
            a += current.heading
            if(a > Math.PI) a -= 2*Math.PI
            if(a < -Math.PI) a += 2*Math.PI
            return a
        }

        fun distanceBetweenPoints(point1: Pose2d, point2: Pose2d): Double {
            return sqrt((point2.y - point1.y).pow(2) + (point2.x - point1.x).pow(2))
        }

        fun distanceBetweenPoints(point1: Vector2d, point2: Vector2d): Double = distanceBetweenPoints(
                Pose2d(point1.x, point1.y, 0.0),
                Pose2d(point2.x, point2.y, 0.0)
        )
    }


    //The two mechanically linked motors
    private val shooterMotor1: DcMotorSimple
    private val shooterMotor2: DcMotorSimple

    //The encoder is actually on motor 2, but we treat it as separate
    val shooterWheelEncoder: DcMotorEx

    //Servo to control angle of shooter, calibrate so that 0 on the servo is in line with the horizontal.
    val shooterAngleServo: Servo



    //The lowest the shooter can go in degrees
    val minumumShooterAngle: Double = 25.0

    //The highest the shooter can go in degrees
    val maximumShooterAngle: Double = 60.0



    //The current target position for the shooter to angle for
//    private var shooterTarget: Vector2d = AllianceTarget.REMOTE.highGoal;


    //var to store the control scheme of the shooter
    private var angleMode: AngleMode = AngleMode.MANUAL


    //Boolean to tell when the flywheel should be on
    private var flywheelOn: Boolean = false

    //The PIDF controller for the flywheel velocity

    /*
    Since we have two motors and one encoder for this, we have to be a little creative. The
    encoder will be on one of the motors, which are mechanically linked together. The input for
    the controller will be that encoder's velocity, and the output will go to both motors.

    We might need some logic that allows the motors to cut power with a toggle control from the
    driver.

     */

    private val flywheelVeloController = PIDFController(flywheelVeloCoefficients, flywheelVelocitykV)


    //The Look-Up Table (LUT) that gives an angle for a distance
    private val shooterAngleLUT = InterpLUT()


    init {
        shooterMotor1 = hardwareMap.get(DcMotorSimple::class.java, HardwareNames.Shooter.SHOOTER_MOTOR_1)
        shooterMotor2 = hardwareMap.get(DcMotorSimple::class.java, HardwareNames.Shooter.SHOOTER_MOTOR_2)

        shooterWheelEncoder = hardwareMap.get(DcMotorEx::class.java, HardwareNames.Shooter.SHOOTER_ENCODER)

        shooterAngleServo = hardwareMap.get(Servo::class.java, HardwareNames.Shooter.SHOOTER_ANGLE_SERVO)

        shooterWheelEncoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        flywheelVeloController.setOutputBounds(-1.0, 1.0)
        flywheelVeloController.targetVelocity = flywheelTargetVelocity

        //Add all the measured values to the LUT
        createLUTValues().forEach { entry -> shooterAngleLUT.add(entry.key, entry.value)  }

        //Generate the LUT
        shooterAngleLUT.createLUT()

    }



    fun update(){

        //Give the velocity controller the current readings, and get an output
        val flywheelMotorOutput = flywheelVeloController.update(shooterWheelEncoder.currentPosition.toDouble(), shooterWheelEncoder.velocity)

        if(flywheelOn){
            shooterMotor1.power = flywheelMotorOutput
            shooterMotor2.power = flywheelMotorOutput
        }

        //TODO: Uncomment this when we have adjustable angle shooter
//
//        val shooterTargetDistance: Double = distanceBetweenPoints(Odometry.world_pose, Pose2d(shooterTarget.x, shooterTarget.y, 0.0))
//
//        if(angleMode == AngleMode.AUTO){
//            setShooterToAngle(shooterAngleLUT.get(shooterTargetDistance))
//        }


    }

    fun turnOnFlywheel(flywheelOn: Boolean){
        this.flywheelOn = flywheelOn
        flywheelVeloController.reset()
    }

    fun setToManualAngle(){
        angleMode = AngleMode.MANUAL
    }

    fun setToAutoAngle(){
        angleMode = AngleMode.AUTO
    }


    //Sets the auto-aim position for the shooter
//    fun setTargetPosition(target: Vector2d){
//        shooterTarget = target
//    }


    fun changeShooterAngleBy(change: Double){
        if(angleMode == AngleMode.MANUAL) {
            val newAngle = Range.scale(shooterAngleServo.position, 0.0, 1.0, 0.0, 360.0) + change
            if (!((newAngle <= minumumShooterAngle) || (newAngle >= maximumShooterAngle))) {
                setShooterToAngle(newAngle)
            }
        }
    }

    //Scale the given angle to a position of the servo
    fun setShooterToAngle(angle: Double){
        shooterAngleServo.position = Range.scale(angle, 0.0, 360.0, 0.0, 1.0)
    }



    //Returns a map that has the values for the LUT
    //TODO: Measure this
    private fun createLUTValues(): HashMap<Double, Double> {
        val valueMap = HashMap<Double, Double>()
        //enter values here in the following format: valueMap[angle of shooter] = distance from high goal
        valueMap[90.0] = 0.0
        return valueMap
    }



}