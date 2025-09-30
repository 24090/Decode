package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.util.LinkedList

class DerivativeCalculator(val storeCount: Int) {
    private var pastStates = LinkedList<Pair<Double, Double>>();

    fun update(newX: Number, newY: Number) = update(Pair(newX, newY))

    fun update(newState: Pair<Number, Number>){
        if (pastStates.size >= storeCount){
            pastStates.removeAt(0)
        }
        pastStates.add(Pair(newState.first.toDouble(), newState.second.toDouble()))
    }
    fun get() = (pastStates[0].second - pastStates[pastStates.size - 1].second) / (pastStates[0].first - pastStates[pastStates.size - 1].first)
}