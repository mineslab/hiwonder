package com.example.armpiprocontrol;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.Vibrator;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.webkit.WebView;
import android.widget.Button;

import com.example.armpiprocontrol.connect.JWebSocketClient;
import com.example.armpiprocontrol.utils.JSONutils;

import org.json.JSONObject;

import java.math.BigDecimal;
import java.net.URI;
import java.util.Timer;
import java.util.TimerTask;

/**
 * 公司名称：深圳市幻尔科技
 * 官网地址：www.hiwonder.com
 * 天猫店铺：hiwonder.tmall.com
 **/

public class Control extends TitleActivity {
    private String url;
    private String _deviceIp = "";
    private WebView mVideoView;
    JWebSocketClient client;
    private boolean stopFlag = false;
    private boolean resetingFlag = false;//正在回中不能发送指令
    private Handler mHandler;

    public static final int MSG_MOVE_AHEAD = 1;
    public static final int MSG_MOVE_BACK = MSG_MOVE_AHEAD + 1;
    public static final int MSG_MOVE_LEFT = MSG_MOVE_BACK + 1;
    public static final int MSG_MOVE_RIGHT = MSG_MOVE_LEFT + 1;
    public static final int MSG_UP = MSG_MOVE_RIGHT + 1;
    public static final int MSG_DOWN = MSG_UP + 1;

    private Vibrator mVibrator;

    private double normalSpeed = 0.05;
    private int moveIndex = 0;

    private Button[] buttons = new Button[5];

    Timer timer = new Timer();

    private int longTouchTag = -1;

    private double high = -10;

    // car run direction
    private int direction = 0;

    // car run speed level
    private int speed = 200;

    private Button[] turnButtons = new Button[2];

    long timeDown, timeUp;

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

        buttons[0] = findViewById(R.id.forward_btn);
        buttons[1] = findViewById(R.id.backward_btn);
        buttons[2] = findViewById(R.id.leftward_btn);
        buttons[3] = findViewById(R.id.rightward_btn);
        buttons[4] = findViewById(R.id.reset_btn);

        turnButtons[0] = findViewById(R.id.turn_left_btn);
        turnButtons[1] = findViewById(R.id.turn_right_btn);

        for (int i = 0; i < buttons.length; i++) {
            buttons[i].setOnTouchListener(new View.OnTouchListener() {
                @Override
                public boolean onTouch(View v, MotionEvent event) {
                    switch (event.getAction()) {
                        case MotionEvent.ACTION_DOWN:
                            longTouchTag = Integer.valueOf((String) v.getTag());
                            if (longTouchTag == 4) {
                                sendCarCmd(0, 0, 0);
                            } else {
                                switch (longTouchTag) {
                                    //up
                                    case 0:
                                        direction = 90;
                                        break;
                                    //down
                                    case 1:
                                        direction = 270;
                                        break;
                                    //left
                                    case 2:
                                        direction = 180;
                                        break;
                                    //right
                                    case 3:
                                        direction = 0;
                                        break;
                                }
                                timeDown = System.currentTimeMillis();
                                sendCarCmd(speed, direction, 0);
                            }
                            break;
                        case MotionEvent.ACTION_MOVE:
                            break;
                        case MotionEvent.ACTION_UP:
                            timeUp = System.currentTimeMillis();
                            longTouchTag = -1;
                            long time = timeUp - timeDown;
                            if (time >= 500) {
                                sendCarCmd(0, direction, 0);
                            } else {
                                new Thread("sendStopThread") {
                                    @Override
                                    public void run() {
                                        try {
                                            Thread.sleep(500 - time);
                                            sendCarCmd(0, direction, 0);
                                        } catch (Exception e) {
                                            e.printStackTrace();
                                        }
                                    }
                                }.start();
                            }
                            break;
                    }
                    return false;
                }
            });
        }

        for (int i = 0; i < turnButtons.length; i++) {
            turnButtons[i].setOnTouchListener(new View.OnTouchListener() {
                @Override
                public boolean onTouch(View v, MotionEvent event) {
                    int tag = Integer.valueOf((String) v.getTag());
                    int angular = 1;
                    if (tag == 1) {
                        angular = -1;
                    }
                    switch (event.getAction()) {
                        case MotionEvent.ACTION_DOWN:
                            timeDown = System.currentTimeMillis();
                            sendCarCmd(0, 0, angular);
                            break;
                        case MotionEvent.ACTION_UP:
                            timeUp = System.currentTimeMillis();
                            long time = timeUp - timeDown;
                            if (time >= 500) {
                                sendCarCmd(0, 0, 0);
                            } else {
                                new Thread("sendStopThread") {
                                    @Override
                                    public void run() {
                                        try {
                                            Thread.sleep(500 - time);
                                            sendCarCmd(0, 0, 0);
                                        } catch (Exception e) {
                                            e.printStackTrace();
                                        }
                                    }
                                }.start();
                            }
                            break;
                    }
                    return false;
                }
            });
        }
        mVibrator = (Vibrator) this.getSystemService(Context.VIBRATOR_SERVICE);
        setTimer();
    }

    @Override
    protected void onResume() {
        url = "http://";
        url += _deviceIp;
        url += ":8080/stream?topic=/usb_cam/image_raw";
        mVideoView.loadUrl(url);
        stopFlag = false;
        URI uri = URI.create("ws://" + _deviceIp + ":9090");
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
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }.start();
        super.onResume();
    }

    @Override
    protected void onPause() {
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

    /**
     * 震动一下
     */
    private void playVibrator() {
        if (mVibrator != null)
            mVibrator.vibrate(30);
    }


    private void sendCarCmd(int speed, int direction, double angular) {
        try {
            JSONObject cmdInfo = new JSONObject();
            cmdInfo.put("op", "publish");
            cmdInfo.put("type", "/chassis_control/SetVelocity");
            cmdInfo.put("topic", "/chassis_control/set_velocity");
            JSONObject argInfo = new JSONObject();
            argInfo.put("velocity", speed);
            argInfo.put("direction", direction);
            argInfo.put("angular", angular);
            cmdInfo.put("msg", argInfo);
            Log.i("hiwonder8", cmdInfo.toString());
            if (client != null && client.isOpen())
                client.send(cmdInfo.toString());
        } catch (Exception ex) {

        }
    }

    private double getRadians(int angle) {
        return angle / 57.3;
    }

    public void setTimer() {
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                if (stopFlag)
                    return;
                if (longTouchTag != -1) {
                    Message msg = new Message();
                    moveIndex = longTouchTag;
                    switch (longTouchTag) {
                        case 0:
                            msg.what = MSG_MOVE_AHEAD;
                            break;
                        case 1:
                            msg.what = MSG_MOVE_BACK;
                            break;
                        case 2:
                            msg.what = MSG_MOVE_LEFT;
                            break;
                        case 3:
                            msg.what = MSG_MOVE_RIGHT;
                            break;
                        case 5:
                            msg.what = MSG_UP;
                            break;
                        case 6:
                            msg.what = MSG_DOWN;
                            break;
                    }
                    mHandler.sendMessage(msg);
                }
            }
        }, 0, 50);
    }

    class MsgCallBack implements Handler.Callback {
        @SuppressLint("ResourceAsColor")
        @Override
        public boolean handleMessage(Message msg) {
            switch (msg.what) {
            }
            return true;
        }
    }
}