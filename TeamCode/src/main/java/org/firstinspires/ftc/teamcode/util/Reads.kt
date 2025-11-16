package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap

class Reads(hwMap: HardwareMap) {
    val bulkReads = BulkReads(hwMap)
    val updateVoltages = VoltageReader.updater(hwMap)
    var n = 0
    fun update(){
        n += 1
        bulkReads.update()
        if (n%10 == 0) updateVoltages()
    }

}