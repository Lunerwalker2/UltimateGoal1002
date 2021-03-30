package org.firstinspires.ftc.teamcode.Robot

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.TeleOp.JankWoodTeleOp

object DriveFields {
    @JvmField
    var movement_x = 0.0
    @JvmField
    var movement_y = 0.0
    @JvmField
    var movement_turn = 0.0


    const val VX_WEIGHT = 1.0
    const val VY_WEIGHT = 1.0
    const val OMEGA_WEIGHT = 1.0

    const val lateralMultiplier = 1.0

    @JvmField
    var lf_power = 0.0
    @JvmField
    var lb_power = 0.0
    @JvmField
    var rf_power = 0.0
    @JvmField
    var rb_power = 0.0

    @JvmStatic
    fun distributePowers(powers: DoubleArray) {
        lf_power = powers[0]
        lb_power = powers[1]
        rf_power = powers[2]
        rb_power = powers[3]
    }

    @JvmStatic
    fun distributePowers(powers: List<Double>) {
        lf_power = powers[0]
        lb_power = powers[1]
        rf_power = powers[3]
        rb_power = powers[2]
    }

    @JvmStatic
    fun normalizedVels(): Pose2d {
        val drivePower = Pose2d(movement_y, movement_x, movement_turn) //LEAVE as is

        var vel = drivePower
        if ((Math.abs(drivePower.x) + Math.abs(drivePower.y)
                        + Math.abs(drivePower.heading)) > 1) {
            // re-normalize the powers according to the weights
            val denom: Double = VX_WEIGHT * Math.abs(drivePower.x) + VY_WEIGHT * Math.abs(drivePower.y) + OMEGA_WEIGHT * Math.abs(drivePower.heading)
            vel = Pose2d(
                    VX_WEIGHT * drivePower.x,
                    VY_WEIGHT * drivePower.y,
                    OMEGA_WEIGHT * drivePower.heading
            ).div(denom)
        }
        return vel
    }
}