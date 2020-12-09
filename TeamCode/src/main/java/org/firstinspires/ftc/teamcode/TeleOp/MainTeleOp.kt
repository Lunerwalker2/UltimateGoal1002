package org.firstinspires.ftc.teamcode.TeleOp

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.TeleOp.MainTeleOp.AimMode
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.TeleOp.MainTeleOp
import org.firstinspires.ftc.teamcode.Robot.DriveFields
import org.firstinspires.ftc.teamcode.Robot.Odometry
import org.firstinspires.ftc.teamcode.Robot.Robot
import org.firstinspires.ftc.teamcode.Robot.Shooter

@TeleOp(name = "Main TeleOp")
@Config
class MainTeleOp : LinearOpMode() {



    companion object {
        //The controller that rotates the robot to aim the shooter automatically
        @JvmField
        var autoAimCoeffs = PIDCoefficients(1.0, 0.0, 0.0)
    }


    //Enum to represent the rotation control of the robot for the shooter; manual really just means "nothing"
    enum class AimMode {
        MANUAL,
        AUTO
    }

    //The (mostly P) controller to control the rotation of the robot to point the shooter
    private lateinit var autoAimController: PIDFController


    //The current target to aim for. This should always be in sync with the shooter target in
    //Shooter.kt, therefore changing this should only be done with changeTarget().
    private var target: Vector2d = Shooter.AllianceTarget.BLUE.highGoal



    //Stores the method of rotating the robot for the shooter
    private var aimMode = AimMode.MANUAL


    //Our robot instance
    private lateinit var robot: Robot


    /**
     * Let's keep a list of the current robot controls here:
     *
     * Gamepad 1 (Driver)
     * left stick x = robot translational
     * left stick y = robot translational
     * right stick x = robot rotational
     *
     *
     *
     *
     * Gamepad 2 (Manipulator)
     *
     *
     */
    @Throws(InterruptedException::class)
    override fun runOpMode() {

        robot = Robot(this, Alliance.TELEOP)


        autoAimController = PIDFController(autoAimCoeffs)
        autoAimController.setOutputBounds(-1.0, 1.0)
        autoAimController.setInputBounds(-Math.PI, Math.PI) //Limits of Euler angles


        waitForStart()


        while (opModeIsActive()) {
            if (isStopRequested) return




            //Give the translation controls of the robot
            DriveFields.movement_x = gamepad1.left_stick_x.toDouble()
            DriveFields.movement_y = gamepad1.left_stick_y.toDouble()


            //Controls the shooter
            if (aimMode == AimMode.AUTO) {
                DriveFields.movement_turn = autoAimController.update(Odometry.world_r) //Update the controller with current angle
            } else {
                DriveFields.movement_turn = gamepad1.right_stick_x.toDouble() //Just pass through normal control
            }



            robot.update()
        }
    }

    //Switch the position that both the shooter (vertically) and the robot (rotationally) aim for automatically
    private fun changeTarget(newTarget: Vector2d){
        robot.shooter.setTargetPosition(newTarget)
        autoAimController.targetPosition = Shooter.angleToTarget(Odometry.world_pose, Pose2d(newTarget, 0.0))
        autoAimController.reset()
        target = newTarget
    }

}