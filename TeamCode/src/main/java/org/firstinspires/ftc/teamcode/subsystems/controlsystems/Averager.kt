package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.lang.Double.sum
import java.util.LinkedList

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
    fun get() = if (pastStates.isEmpty()) 0.0 else pastStates.sum()/ pastStates.size
    fun deriv() = if(pastStates.size<10) 0.0 else (pastStates[pastStates.size-1]-pastStates[pastStates.size-10])/(thisTime-lastTime)
}