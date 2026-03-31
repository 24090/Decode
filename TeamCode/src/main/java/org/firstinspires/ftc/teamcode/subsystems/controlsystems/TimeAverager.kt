package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import com.rathippo.commandviewer.CommandMessage
import java.lang.Exception
import java.util.LinkedList


class TimeAverager(val storeCount: Int) {
    private var pastStates = LinkedList<Pair<Double, Double>>()

    fun update(newX: Number, newY: Number) = update(Pair(newX, newY))

    fun update(newState: Pair<Number, Number>) {
        if (pastStates.size >= storeCount) {
            pastStates.removeAt(0)
        }
        pastStates.add(Pair(newState.first.toDouble(), newState.second.toDouble()))
    }

    fun get(): Double {
        if (pastStates.size < 2) return 0.0
        val sum = pastStates.take(pastStates.size - 1)
            .zip(
                pastStates.drop(pastStates.size - 1)
                    .map(Pair<Double, Double>::first)
            ).sumOf { (pair, t1) -> (pair.first - t1) * pair.second }
        return try { sum/(pastStates[0].first - pastStates.last().first) } catch (e: Exception) { 0.0 }
    }
}