package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.util.TreeMap

class InterpolatedLUT(items: Map<Double, Double>) {
    var map = TreeMap<Double, Double>(items)
    fun get(v: Double): Double{
        val floorEntry = map.floorEntry(v)
        val ceilingEntry = map.ceilingEntry(v)
        if (floorEntry == null){
            return ceilingEntry.value
        }
        if (ceilingEntry == null){
            return floorEntry.value
        }
        val slope = (floorEntry.value - ceilingEntry.value)/(floorEntry.key - ceilingEntry.key)
        return (slope*(v - floorEntry.key) + floorEntry.value)
    }
}