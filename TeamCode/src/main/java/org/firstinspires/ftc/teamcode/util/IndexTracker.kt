package org.firstinspires.ftc.teamcode.util

import kotlin.math.min

enum class BallColor {
    GREEN,
    PURPLE,
}

fun patternFromG(n: Int) = when (n%3) {
    0 -> Pattern.GPP
    1 -> Pattern.PGP
    2 -> Pattern.PPG
    else -> throw Error("Unreachable")
}
enum class Pattern(val n: Int) {
    GPP(0),
    PGP(1),
    PPG(2)
}

sealed class Observation {
    class Camera(val num: Int) : Observation()
    class Shot(val n: Int) : Observation()
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
                rampCount + observation.n
            }
            is Observation.GateOpened -> {
                0
            }
        }
    }

    fun getRecommendations() = patternFromG(pattern.n - rampCount)
}