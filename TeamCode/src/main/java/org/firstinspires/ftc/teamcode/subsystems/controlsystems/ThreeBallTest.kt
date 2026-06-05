package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.util.Collections.max
import java.util.Collections.min
import java.util.LinkedList
import kotlin.math.max
import kotlin.math.min

class ThreeBallTest(val storeCount: Int) {
    private var pastStates = LinkedList<Double>()
    var lastLeft: Int = 10
    var lastRight: Int = 10
    var baseline: Double = 0.0
    fun isStalling() = (min(pastStates.takeLast(min(5, pastStates.size))+listOf(1500.0)) < 1050) && lastLeft < 10 && lastRight < 10 && increasing() < 300
    fun increasing() = max(pastStates.takeLast(min(5, pastStates.size))) - min(pastStates.takeLast(min(25, pastStates.size)))
    fun update(newState: Number, newLeftDetected: Boolean, newRightDetected: Boolean, time: Long){
        if (newRightDetected && lastLeft < 10 && lastRight > 10){
            baseline = newState.toDouble()
        }
        if (newLeftDetected && lastLeft < 10 && lastRight > 10){
            baseline = newState.toDouble()
        }
        lastLeft = if (newLeftDetected) 0 else lastLeft+1
        lastRight = if (newRightDetected) 0 else lastRight+1

        pastStates.add(newState.toDouble())
    }
    fun getAvg(n: Int) = if (pastStates.isEmpty()) 0.0 else pastStates.takeLast(min(n, pastStates.size)).sum()/min(n, pastStates.size)

}