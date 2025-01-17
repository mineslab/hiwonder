package com.example.armpiprotag;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.webkit.WebView;
import android.widget.Toast;

import com.example.armpiprotag.connect.JWebSocketClient;
import com.example.armpiprotag.utils.JSONutils;
import com.suke.widget.SwitchButton;

import java.net.URI;
import java.util.Timer;
import java.util.TimerTask;
/**
 *公司名称：深圳市幻尔科技
 *官网地址：www.hiwonder.com
 *天猫店铺：hiwonder.tmall.com
 **/
public class TagRecognition extends TitleActivity {
    private String url;
    private String _deviceIp = "";
    private WebView mVideoView;
    JWebSocketClient client;
    private Handler mHandler;
    private boolean stopFlag = false;
    private SwitchButton switchButton;
    public static final int MSG_TWO_SECOND = 1;
    Timer timer = new Timer();
    private String service = "apriltag_detect";
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_control);
        showLeftBtn(true);
        showRightBtn(false);
        mHandler = new Handler(new MsgCallBack());

        Intent intent = getIntent();
        _deviceIp = intent.getStringExtra("deviceip");
        mVideoView = (WebView) findViewById(R.id.video);
        double ratioH = (MainActivity.screenHigh - 160) * 100.0 / 480.0;
        double ratioW = MainActivity.screenWidth * 100.0 / (640.0 * 1.6);
        double ratio = Math.min(ratioH, ratioW);
        mVideoView.setInitialScale((int) ratio);

        switchButton = findViewById(R.id.switch_button);
        switchButton.setOnCheckedChangeListener(new SwitchButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(SwitchButton view, boolean isChecked) {
                if (isChecked) {
                    if (client != null && client.isOpen())
                    {
                        client.send(JSONutils.sendRunning(service, true));//开启玩法
                        Toast.makeText(getBaseContext(), R.string.start, Toast.LENGTH_SHORT).show();
                    }
                } else {
                    if (client != null && client.isOpen())
                    {
                        client.send(JSONutils.sendRunning(service, false));//关闭玩法
                        Toast.makeText(getBaseContext(), R.string.stop, Toast.LENGTH_SHORT).show();
                    }
                }
            }
        });
        SetTimer();
    }

    @Override
    protected void onResume() {
        url = "http://";
        url += _deviceIp;
        url += ":8080/stream?topic=/visual_processing/image_result";
        mVideoView.loadUrl(url);
        stopFlag = false;
        URI uri = URI.create("ws://" + _deviceIp + ":9090");
        switchButton.setChecked(false);
        client = new JWebSocketClient(uri) {
            @Override
            public void onMessage(String message) {
                //message就是接收到的消息
                Log.e("JWebSClientService", message);
            }
        };
        new Thread("sendThread") {
            @Override
            public void run() {
                try {
                    client.connectBlocking();
                    //进入玩法
                    if (client != null && client.isOpen()) {
                        client.send(JSONutils.modeEnter(service));
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }.start();
        super.onResume();
    }


    @Override
    protected void onPause() {
        if (client != null && client.isOpen())
            client.send(JSONutils.modeExit(service));
        disconnectAllSocket();
        stopFlag = true;
        super.onPause();
    }

    @Override
    protected void onStop() {
        super.onStop();
    }

    @Override
    protected void onDestroy() {
        mVideoView.loadDataWithBaseURL(null, "", "text/html", "utf-8", null);
        mVideoView.clearCache(true);
        mVideoView.destroy();
        mVideoView = null;
        timer.cancel();
        mHandler.removeCallbacksAndMessages(null);
        super.onDestroy();
    }

    private void disconnectAllSocket() {
        try {
            if (null != client) {
                client.close();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            client = null;
        }
    }

    private void SetTimer()//设置定时
    {
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                if (stopFlag)
                    return;
                Message messageHalf = new Message();
                messageHalf.what = MSG_TWO_SECOND;
                mHandler.sendMessage(messageHalf);
            }
        }, 0, 2000);
    }

    class MsgCallBack implements Handler.Callback {
        @SuppressLint("ResourceAsColor")
        @Override
        public boolean handleMessage(Message msg) {
            switch (msg.what) {
                case MSG_TWO_SECOND:
                    Log.d("hiwonder", "heartbeat");
                    //发送心跳包
                    if (client != null && client.isOpen()) {
                        client.send(JSONutils.sendHeartBeat(service));
                    }
                    break;
            }
            return true;
        }
    }
}