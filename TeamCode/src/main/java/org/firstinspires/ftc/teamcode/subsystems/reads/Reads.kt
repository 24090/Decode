package org.firstinspires.ftc.teamcode.subsystems.reads

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap

class Reads(hwMap: HardwareMap, val readPinpoint: Boolean = true, val readVoltages: Boolean = true, val bulkRead: Boolean = true) {
    val bulkReads = BulkReads(hwMap)
    val updateVoltages = VoltageReader.updater(hwMap)
    val pinpoint: GoBildaPinpointDriver = hwMap.get(GoBildaPinpointDriver::class.java, "pinpoint")
    var n = 0
    fun update(){
        n += 1
        if (readPinpoint) pinpoint.update()
        if (bulkRead) bulkReads.update()
        if (n%10 == 0 && readVoltages) updateVoltages()
    }

}