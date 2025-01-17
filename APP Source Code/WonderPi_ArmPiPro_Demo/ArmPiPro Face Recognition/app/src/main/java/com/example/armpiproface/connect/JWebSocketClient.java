package com.example.armpiproface.connect;

import android.util.Log;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.drafts.Draft;
import org.java_websocket.handshake.ServerHandshake;

import java.net.URI;
import java.util.Map;

/**
 * created by hiwonder
 * on 2021/1/25
 * Explanation :
 */
public class JWebSocketClient extends WebSocketClient {

    public JWebSocketClient(URI serverUri, Draft draft) {
        super(serverUri, draft);
    }

    public JWebSocketClient(URI serverUri) {
        super(serverUri);
    }

    public JWebSocketClient(URI serverUri, Map<String, String> httpHeaders) {
        super(serverUri, httpHeaders);
    }

    @Override
    public void onOpen(ServerHandshake handshakedata) {
        Log.e("JWebSocketClient", "onOpen()");
    }

    @Override
    public void onMessage(String message) {
        Log.e("JWebSocketClient", "onMessage()");
    }

    @Override
    public void onClose(int code, String reason, boolean remote) {
        Log.e("JWebSocketClient", "onClose()");
    }

    @Override
    public void onError(Exception ex) {
        Log.e("JWebSocketClient", "onError()");
    }
}
