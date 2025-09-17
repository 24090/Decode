package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.util.LinkedList

class DerivativeCalculator {
    val store_count = 3;
    private var pastStates = LinkedList<Pair<Double, Double>>();
    fun update(new_state: Pair<Double, Double>){
        if (pastStates.size >= store_count){
            pastStates.removeAt(0)
        }
        pastStates.add(new_state)
        (pastStates[0].second - pastStates[pastStates.size - 1].second) / (pastStates[0].first - pastStates[pastStates.size - 1].first)
    }
}