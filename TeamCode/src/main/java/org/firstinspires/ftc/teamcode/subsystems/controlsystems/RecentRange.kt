package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import org.firstinspires.ftc.teamcode.util.clamp
import java.util.LinkedList
import kotlin.math.abs
import kotlin.math.max

class SpikyTest(val storeCount: Int, val spikeThresh: Double = 40.0) {
    private var pastStates = LinkedList<Double>()

    fun update(newState: Number){
        if (pastStates.size >= storeCount){
            pastStates.removeAt(0)
        }
        pastStates.add(newState.toDouble())
    }
    fun fullStall(): Boolean {
        if( pastStates.size < 3){
            return false
        } else  {
            return (pastStates.sum())/pastStates.size < 50.0
        }
    }
    fun spikeValue(): Double {
        if( pastStates.size < 3){
            return 0.0
        }
        var sum = 0.0
        var last = pastStates[1]
        var lastlast = pastStates[0]
        for (n in (2..pastStates.size - 1)) {
            val d0 = last - lastlast
            val d1 = pastStates[n] - last
            sum += clamp((abs(d1 - d0) - spikeThresh), 0.0, 25.0)
            lastlast = last
            last = pastStates[n]
        }
        return sum
    }
}