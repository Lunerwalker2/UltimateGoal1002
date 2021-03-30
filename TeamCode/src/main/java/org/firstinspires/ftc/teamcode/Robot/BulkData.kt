package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.Util.HardwareNames

class BulkData(robot: Robot): Component(robot) {



    val HUB_LEFT: LynxModule

    val HUB_RIGHT: LynxModule


    init {
        HUB_LEFT = hardwareMap.get(LynxModule::class.java, HardwareNames.Hubs.HUB_LEFT)
        HUB_RIGHT = hardwareMap.get(LynxModule::class.java, HardwareNames.Hubs.HUB_RIGHT)

    }

    /**
     *     Update "everything" lol. Basically just clear the cache
     */
    fun update() {
        clearCache()
    }

    /**
     * Clear the cache
     */
    private fun clearCache() {
        HUB_LEFT.clearBulkCache()
        HUB_RIGHT.clearBulkCache()

    }

    /**
     * Set the mode as MANUAL
     */
    fun setManual() {
        HUB_LEFT.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        HUB_RIGHT.bulkCachingMode =  LynxModule.BulkCachingMode.MANUAL
    }

    /**
     * Set the mode as AUTO
     */
    fun setAuto() {
        HUB_LEFT.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        HUB_RIGHT.bulkCachingMode =  LynxModule.BulkCachingMode.AUTO
    }

    /**
     * Set the mode as OFF
     */
    fun setOff() {
        HUB_LEFT.bulkCachingMode = LynxModule.BulkCachingMode.OFF
        HUB_RIGHT.bulkCachingMode =  LynxModule.BulkCachingMode.OFF
    }

    

}