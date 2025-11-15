package com.rathippo.commandviewer

import android.content.Context
import android.content.res.AssetManager
import com.reidsync.kxjsonpatch.JsonDiff
import fi.iki.elonen.NanoHTTPD.IHTTPSession
import fi.iki.elonen.NanoWSD
import fi.iki.elonen.NanoWSD.WebSocket
import fi.iki.elonen.NanoWSD.WebSocketFrame
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonObject
import java.io.File
import java.io.IOException
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit

internal class Socket(handshakeRequest: IHTTPSession?, private val initialMessage: String) : WebSocket(handshakeRequest) {
    var pingHandle: ScheduledFuture<*>? = null
    override fun onOpen() {
        println("CommandViewer: Websocket Opened")
        send(initialMessage)
        pingHandle = Executors.newScheduledThreadPool(1)
            .scheduleWithFixedDelay({ ping(byteArrayOf()) }, 4, 4, TimeUnit.SECONDS)
    }

    override fun onClose(
        code: WebSocketFrame.CloseCode?,
        reason: String?,
        initiatedByRemote: Boolean,
    ) {
        println("CommandViewer: Websocket Closed")
        println(".              code: $code")
        println(".              reason: $reason")
        println(".              byRemote: $initiatedByRemote")
        pingHandle!!.cancel(true)
    }

    override fun onMessage(webSocketFrame: WebSocketFrame) {
        CommandViewer.runFunction(webSocketFrame.textPayload.toInt())
    }

    override fun onPong(pong: WebSocketFrame?) {
        // pass
    }

    override fun onException(exception: IOException?) {
        println("CommandViewer: Exception in WebSocket $exception")
    }
}

internal class Server(val page: Response) : NanoWSD(24090) {
    private val initialValue: JsonElement = JsonObject(mapOf(
        "messages" to JsonArray(listOf())
    ))
    private var cachedMessage: JsonElement = initialValue
    var sockets: LinkedHashSet<Socket> = LinkedHashSet()
    init {
        start(SOCKET_READ_TIMEOUT, false)
        println("CommandViewer: Server Running")
    }

    fun update(msg: JsonElement){
        if (sockets.isEmpty()) {
            return
        }
        val patch = JsonDiff.asJson(cachedMessage, msg)
        if (patch.isEmpty()){
            cachedMessage = msg
            return
        }
        sockets = LinkedHashSet(sockets.filter { socket ->  socket.isOpen })
        for (socket in sockets){
            if (!socket.isOpen){
                continue
            }
            try {
                socket.send(patch.toString())
            } catch (e: IOException){
                println("CommandViewer: Unable to send message due to: Exception in WebSocket $e")
            }
        }
        cachedMessage = msg
    }

    override fun serveHttp(session: IHTTPSession): Response {
        println("received http request: ${session.uri}")
        if (
            session.uri != "/"
        ){
            return newFixedLengthResponse("404")
        }
        return page
    }

    override fun openWebSocket(ihttpSession: IHTTPSession): WebSocket {
        val socket = Socket(ihttpSession, JsonDiff.asJson(initialValue, cachedMessage).toString())
        sockets.add(socket)
        return socket
    }
}