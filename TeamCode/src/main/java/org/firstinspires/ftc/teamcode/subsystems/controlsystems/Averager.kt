package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.lang.Double.sum
import java.util.LinkedList

class Averager(val storeCount: Int) {
    private var pastStates = LinkedList<Double>()

    fun update(newState: Number){
        if (pastStates.size >= storeCount){
            pastStates.removeAt(0)
        }
        pastStates.add(newState.toDouble())
    }
    fun get() = if (pastStates.isEmpty()) 0.0 else pastStates.sum()/ pastStates.size
}