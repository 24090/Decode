package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.util.LinkedList
import kotlin.math.absoluteValue
import kotlin.math.min

class StallTest(val storeCount: Int) {
    private var pastStates = LinkedList<Double>()
    var lastDetected = 5
    var lastTime: Long = 0
    var thisTime: Long = 0
    var count: Int = 0
    fun isStalling() = (getAvg(80 - count * 30) < 1050)  && (getAvg(80 - count * 30) > 900)
    fun update(newState: Number, time: Long){
        lastTime = thisTime
        val detected = getAvg(4) < 1100 && getAvg(4) > 950 && (pastStates.size > 40)
        if (pastStates.size >= storeCount){
            pastStates.removeAt(0)
        }
        if (detected && (lastDetected > 15) && count < 2){
            count++
        }
        pastStates.add(newState.toDouble())
        thisTime = time
        if (detected) {
            lastDetected = 0
        } else {
            lastDetected++
        }
    }
    fun getAvg(n: Int) = if (pastStates.isEmpty()) 0.0 else pastStates.takeLast(min(n, pastStates.size)).sum()/min(n, pastStates.size)
    fun resetCount() { count = 0 }

}