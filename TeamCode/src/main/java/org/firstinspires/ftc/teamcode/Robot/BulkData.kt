package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.Util.HardwareNames

class BulkData(robot: Robot): Component(robot) {



    val HUB_LEFT: LynxModule

    val HUB_RIGHT: LynxModule

    private val modules: MutableList<LynxModule> = ArrayList(2)

    init {
        HUB_LEFT = hardwareMap.get(LynxModule::class.java, HardwareNames.Hubs.HUB_LEFT)
        HUB_RIGHT = hardwareMap.get(LynxModule::class.java, HardwareNames.Hubs.HUB_RIGHT)

        //add to list for easy access
        modules.add(HUB_LEFT)
        modules.add(HUB_RIGHT)

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
        modules.forEach {module: LynxModule -> module.clearBulkCache()}

    }

    /**
     * Set the mode as MANUAL
     */
    private fun setManual() {
        modules.forEach { module: LynxModule -> module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL   }
    }

    /**
     * Set the mode as AUTO
     */
    fun setAuto() {
        modules.forEach { module: LynxModule -> module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO   }
    }

    /**
     * Set the mode as OFF
     */
    fun setOff() {
        modules.forEach { module: LynxModule -> module.bulkCachingMode = LynxModule.BulkCachingMode.OFF   }
    }

    

}