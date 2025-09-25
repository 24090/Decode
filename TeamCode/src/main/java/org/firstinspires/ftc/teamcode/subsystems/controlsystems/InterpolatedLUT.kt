package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.util.TreeMap

class InterpolatedLUT(items: Map<Double, Double>) {
    var map = TreeMap<Double, Double>(items)
    fun get(v: Double): Double{
        val floorEntry = map.floorEntry(v)
        val ceilingEntry = map.ceilingEntry(v)
        val slope = (floorEntry.value - ceilingEntry.value)/(floorEntry.key - ceilingEntry.value)
        return (slope*(v - floorEntry.value) + ceilingEntry.value)
    }
}