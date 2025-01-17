package com.example.armpiproline;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.webkit.WebView;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.Toast;

import com.example.armpiproline.connect.JWebSocketClient;
import com.example.armpiproline.utils.JSONutils;
import com.suke.widget.SwitchButton;

import org.json.JSONObject;

import java.net.URI;
import java.util.Timer;
import java.util.TimerTask;
/**
 *公司名称：深圳市幻尔科技
 *官网地址：www.hiwonder.com
 *天猫店铺：hiwonder.tmall.com
 **/
public class LineTracking extends TitleActivity {
    private String url;
    private String _deviceIp = "";
    private WebView mVideoView;
    JWebSocketClient client;
    private Handler mHandler;
    private boolean stopFlag = false;
    private SwitchButton switchButton;
    public static final int MSG_TWO_SECOND = 1;
    Timer timer = new Timer();
    private String service = "visual_patrol";
    private RadioGroup radioGroup;
    private RadioButton whiteRadio, redRadio;
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
                        Toast.makeText(getBaseContext(), R.string.ask_select_color, Toast.LENGTH_SHORT).show();
                    }
                    whiteRadio.setEnabled(true);
                    redRadio.setEnabled(true);
                } else {
                    if (client != null && client.isOpen())
                    {
                        client.send(JSONutils.sendRunning(service, false));//关闭玩法
                        Toast.makeText(getBaseContext(), R.string.stop, Toast.LENGTH_SHORT).show();
                    }
                    whiteRadio.setEnabled(false);
                    redRadio.setEnabled(false);
                    radioGroup.clearCheck();
                }
            }
        });
        whiteRadio = findViewById(R.id.radio_white);
        redRadio = findViewById(R.id.radio_red);
        whiteRadio.setEnabled(false);
        redRadio.setEnabled(false);

        radioGroup = findViewById(R.id.radio_group);
        radioGroup.clearCheck();
        radioGroup.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                if (switchButton.isChecked()) {
                    if (checkedId == R.id.radio_white) {
                        sendColorCmd("white");
                    }
                    else if (checkedId == R.id.radio_red) {
                        sendColorCmd("red");
                    }
                }
                else {
                    Toast.makeText(getBaseContext(), R.string.please_open_line_tracking, Toast.LENGTH_SHORT).show();
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

    public void sendColorCmd(String color)
    {
        try
        {
            JSONObject cmdInfo = new JSONObject();
            cmdInfo.put("op", "call_service");
            cmdInfo.put("type", "/visual_patrol/SetTarget");
            cmdInfo.put("service", "/visual_patrol/set_target");
            JSONObject argInfo = new JSONObject();
            argInfo.put("data", color);
            cmdInfo.put("args", argInfo);
            System.out.println(cmdInfo.toString());
            if (client != null && client.isOpen())
                client.send(cmdInfo.toString());
        }
        catch (Exception ex)
        {

        }
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