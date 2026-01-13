package org.firstinspires.ftc.teamcode.util

fun Boolean.toDouble() = if (this) 1.0 else 0.0

class Reference<T>(private var value: T){
    fun get() = value
    fun set(newValue: T) {
        this.value = newValue
    }
}