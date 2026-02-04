package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import org.firstinspires.ftc.teamcode.util.clamp
import java.util.LinkedList
import kotlin.math.abs
import kotlin.math.pow

class StallTest(val storeTime: Int, val lookTime: Int) {
    private var pastStates = LinkedList<Pair<Int, Int>>()

    fun update(newTime: Long, newValue: Int){
        while (!pastStates.isEmpty() && ((newTime - pastStates[0].first) >= storeTime)){
            pastStates.removeAt(0)
        }
        pastStates.add(Pair(newTime.toInt(), newValue))
    }

    fun weight(distance: Double) =
        if (abs(distance) > lookTime)
            0.0
        else
            (1.0-abs(distance/lookTime).pow(3)).pow(3)

    fun spikeValue(): Double {
        if (pastStates.isEmpty()){
            return 0.0
        }

        var sum = 0.0
        var start = 0
        var end = 0
        for (look in 0..pastStates.size - 1){
            val time = pastStates[look].first
            while ((pastStates[end].first - time < lookTime) && (end < pastStates.size - 1)){
                end += 1
            }
            while (time - pastStates[start].first > lookTime){
                start += 1
            }
            val weights = pastStates.subList(start, end).map { (a, b) -> weight((time - a).toDouble()) }
            println("${weights.sum()}")
            val average = weights.mapIndexed { index, value -> pastStates[start + index].second.toDouble() * value }.sum()/weights.sum()
            val error = clamp(abs((average - pastStates[look].second.toDouble())) - 10, 0.0, 5000.0)
            sum += error * (pastStates[clamp(look + 1, 0, pastStates.size -1)].first - pastStates[clamp(look - 1, 0, pastStates.size -1)].first)
        }
        return sum/(pastStates.last().first - pastStates.first().first)
    }
}