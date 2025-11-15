package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.hardware.HardwareMap


class BulkReads(hardwareMap: HardwareMap){
    private val lynxModules = hardwareMap.getAll(LynxModule::class.java)
    init {
        for (module in lynxModules) {
            module.bulkCachingMode = BulkCachingMode.MANUAL
        }
    }
    fun update(){
        for (module in lynxModules){
            module.clearBulkCache()
        }
    }

}