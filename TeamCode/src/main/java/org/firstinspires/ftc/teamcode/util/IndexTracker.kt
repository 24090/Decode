package org.firstinspires.ftc.teamcode.util

import kotlin.math.min

enum class BallColor {
    GREEN,
    PURPLE,
}

enum class Pattern(val n: Int) {
    GPP(0),
    PGP(1),
    PPG(2)
}

sealed class Observation {
    class Camera(val num: Int) : Observation()
    object Shot : Observation()
    object GateOpened : Observation()
}

class IndexTracker(){
    var pattern = Pattern.GPP
    var rampCount = 0

    fun processObservation(observation: Observation){
        rampCount = when (observation) {
            is Observation.Camera -> {
                min(rampCount, observation.num)
            }
            is Observation.Shot -> {
                rampCount + 1
            }
            is Observation.GateOpened -> {
                0
            }
        }
    }

    fun getRecommendations() = Pair(
        if (rampCount%3 == pattern.n) BallColor.GREEN else BallColor.PURPLE,
        if ((rampCount+1)%3 == pattern.n) BallColor.GREEN else BallColor.PURPLE
    )
}