package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.util.LinkedList
import kotlin.math.min

class Averager(val storeCount: Int) {
    private var pastStates = LinkedList<Double>()
    var lastTime: Long = 0
    var thisTime: Long = 0
    fun update(newState: Number, time: Long){
        lastTime = thisTime
        if (pastStates.size >= storeCount){
            pastStates.removeAt(0)
        }
        pastStates.add(newState.toDouble())
        thisTime = time
    }
    fun get() = if (pastStates.isEmpty()) 0.0 else pastStates.takeLast(min(35, pastStates.size)).sum()/min(35, pastStates.size)
    fun deriv() = if(pastStates.size<20) 0.0 else (pastStates[pastStates.size-1]-pastStates[pastStates.size-20])/(thisTime-lastTime)

    fun derivSpike() = this.deriv() > 17

    fun rememberStall() = pastStates.map { (it < 700) }.reduceOrNull { a, b -> a || b } ?: false

}