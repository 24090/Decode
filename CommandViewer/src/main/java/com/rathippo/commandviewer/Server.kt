package com.rathippo.commandviewer

import com.rathippo.commandviewer.Server.sockets
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
        sockets.remove(this)
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

internal object Server : NanoWSD(24090) {
    private val initialValue: JsonElement = JsonObject(mapOf(
        "messages" to JsonArray(listOf())
    ))
    private var cachedMessage: JsonElement = initialValue
    const val WEB_DIRECTORY = "./CommandViewer/src/main/web"
    var sockets: HashSet<Socket> = HashSet()
    init {
        start(SOCKET_READ_TIMEOUT, false)
        println("CommandViewer: Server Running")
    }

    fun update(msg: JsonElement){
        val patch = JsonDiff.asJson(cachedMessage, msg)
        if (patch.isEmpty()){
            cachedMessage = msg
            return
        }
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
        return newFixedLengthResponse(
            File("$WEB_DIRECTORY/main.html").readText().replace(
                "@INSERT_JSONPATCH",
                File("$WEB_DIRECTORY/jsonpatch.min.js").readText().split("\n").last()
            ).replace(
                "@INSERT_JS",
                File("$WEB_DIRECTORY/main.js").readText()
            ).replace(
                "@INSERT_CSS",
                File("$WEB_DIRECTORY/style.css").readText()
            ).replace(
                "@INSERT_CUSTOMCSS",
                CommandViewerParams.customCss
            )
        )
    }

    override fun openWebSocket(ihttpSession: IHTTPSession): WebSocket {
        val socket = Socket(ihttpSession, JsonDiff.asJson(initialValue, cachedMessage).toString())
        sockets.add(socket)
        return socket
    }
}