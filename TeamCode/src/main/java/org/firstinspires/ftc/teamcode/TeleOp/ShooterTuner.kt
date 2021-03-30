package org.firstinspires.ftc.teamcode.TeleOp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Util.HardwareNames
import org.firstinspires.ftc.teamcode.Util.Toggle
import kotlin.jvm.Throws


@Config
@TeleOp
class ShooterTuner: LinearOpMode() {


    val dashboard = FtcDashboard.getInstance()


    companion object {


        //The target velocity for the flywheel when it is on. In terms of ticks per rev of the encoder
        //2000 RPM, rev hd hex encoder shaft is 28 ticks per rev
        @JvmField
        var flywheelTargetVelocity = 800.0; //TODO: Find this

        @JvmField
        var flywheelMaxVelocity = 1000.0; //i had to choose a number

        @JvmField
        var flywheelVeloCoefficients = PIDCoefficients(0.01, 0.0, 0.0)

        @JvmField
        var flywheelVelocitykV: Double = 0.001

        @JvmField
        var flywheelVelocityKa: Double = 0.002 //arbitrary

        @JvmField
        var flywheelVelocityKstatic: Double = 0.05
    }



    //The two mechanically linked motors
    private lateinit var shooterMotor1: DcMotorSimple
    private lateinit var shooterMotor2: DcMotorSimple

    //The encoder is actually on motor 2, but we treat it as separate
    private lateinit var shooterWheelEncoder: DcMotorEx


    //Boolean to tell when the flywheel should be on
    private var flywheelOn: Boolean = false

    private var flywheelVeloController = PIDFController(
            flywheelVeloCoefficients,
            flywheelVelocitykV
    )

    private var packet = TelemetryPacket()

    private var flywheelBounce = false
    private var flywheelChanged = false


    @Throws(InterruptedException::class)
    override fun runOpMode() {


        shooterMotor1 = hardwareMap[DcMotorSimple::class.java, HardwareNames.Shooter.SHOOTER_MOTOR_1]
        shooterMotor2 = hardwareMap[DcMotorSimple::class.java, HardwareNames.Shooter.SHOOTER_MOTOR_2]

        /*
        This is the internal encoder of a shooter motor, however it is on the motor port of
        another motor due to our dead wheel encoders. As long as we never mess with the motor, using
        it like this is fine.
         */
        shooterWheelEncoder = hardwareMap[DcMotorEx::class.java, HardwareNames.Shooter.SHOOTER_ENCODER]

        shooterWheelEncoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        //Never change this
        shooterWheelEncoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        flywheelVeloController.targetVelocity = 0.0


        waitForStart()

        flywheelVeloController.reset()

        while (opModeIsActive()){
            if(isStopRequested) return

            flywheelVeloController = PIDFController(
                    flywheelVeloCoefficients,
                    flywheelVelocitykV
            )

            packet = TelemetryPacket()


            if(gamepad2.left_bumper && !flywheelBounce){
                changeFlywheel();
                flywheelBounce = true;
            } else if(!gamepad2.left_bumper){
                flywheelBounce = false;
            }


            //Give the velocity controller the current readings, and get an output
            var flywheelMotorOutput = flywheelVeloController.update(shooterWheelEncoder.currentPosition.toDouble(), shooterWheelEncoder.velocity)



            if(flywheelOn){
                shooterMotor1.power = -flywheelMotorOutput
                shooterMotor2.power = -flywheelMotorOutput
            } else {
                shooterMotor1.power = 0.0
                shooterMotor2.power = 0.0
            }

            packet.put("Target Velocity", if(flywheelOn) flywheelTargetVelocity else 0.0)
            packet.put("Measured Velocity", shooterWheelEncoder.velocity)

            dashboard.sendTelemetryPacket(packet)

        }


    }

    private fun changeFlywheel(){
        if(!flywheelChanged){
            flywheelOn = true
            flywheelVeloController.reset()
            flywheelVeloController.targetVelocity = flywheelTargetVelocity
            flywheelChanged = true
        } else {
            flywheelOn = false
            flywheelVeloController.targetVelocity = 0.0
            flywheelChanged = false
        }
    }
}