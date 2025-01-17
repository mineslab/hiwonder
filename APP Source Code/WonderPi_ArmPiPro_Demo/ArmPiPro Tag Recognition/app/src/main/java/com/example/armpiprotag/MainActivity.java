package com.example.armpiprotag;

import android.app.AlertDialog;
import android.content.Intent;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.TextView;
import android.widget.Toast;

import com.example.armpiprotag.component.HorizontalListView;
import com.example.armpiprotag.component.HorizontalListViewAdapter;
import com.example.armpiprotag.connect.Scanner;
import com.example.armpiprotag.dialog.WifiDeviceInfoDialog;

import java.net.InetAddress;
import java.util.HashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;
/**
 *公司名称：深圳市幻尔科技
 *官网地址：www.hiwonder.com
 *天猫店铺：hiwonder.tmall.com
 **/
public class MainActivity extends TitleActivity  {

    public static Scanner _scanner;

    private AlertDialog dialog;

    private boolean confirm;
    HorizontalListView hListView;
    public static HorizontalListViewAdapter hListViewAdapter;
    private TextView searchingInfoText;
    public static boolean needScan = true;

    public static String WIFI_Name;
    private boolean canSeePas = false;

    Intent intent;

    public static String Password;
    public static int model = 1;//直连模式1；局域网模式2；

    public static int screenWidth;
    public static int screenHigh;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
//        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.activity_main);
        showRightBtn(true);
        RefreshBtnAnimStart(true);
        DisplayMetrics mDisplayMetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(mDisplayMetrics);
        screenWidth = mDisplayMetrics.widthPixels;
        screenHigh = mDisplayMetrics.heightPixels;

        hListView = (HorizontalListView) findViewById(R.id.horizon_listview);
        hListViewAdapter = new HorizontalListViewAdapter(this);
        hListView.setAdapter(hListViewAdapter);
        searchingInfoText = (TextView) findViewById(R.id.searchInfoText);
        _scanner = new Scanner(this);

        _scanner.setOnScanOverListener(new Scanner.OnScanOverListener() {
            @Override
            public void onResult(Map<InetAddress, String> data, InetAddress gatewayAddress) {

                if (data != null) {//扫描到的设备
                    for (Map.Entry<InetAddress, String> entry : data.entrySet()) {
                        String id = entry.getValue();
                        String ip = entry.getKey().getHostAddress();

                        for (int i = 0; i < hListViewAdapter.getCount(); i++) {
                            HashMap<String, String> mapSearch = hListViewAdapter.getItem(i);
                            if (mapSearch.get("item_id").equals(id)) {
                                hListViewAdapter.remove(i);
                            }
                        }
                        //添加到列表
                        HashMap<String, String> map = new HashMap<String, String>();
                        map.put("item_id", id);
                        map.put("item_ip", ip);
                        hListViewAdapter.add(map);
                    }
                    searchingInfoText.setText(R.string.device_list);
                }
                if (hListViewAdapter.getCount() == 0) {
                    searchingInfoText.setText(R.string.no_find_device);
                    Toast.makeText(getBaseContext(), R.string.no_find_device, Toast.LENGTH_SHORT).show();
                }

                RefreshBtnAnimStart(false);

//                final Intent intent = new Intent();
//                intent.setClass(getBaseContext(), FaceRecognition.class);
//                startActivity(intent);
            }
        });

        hListView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                final HashMap<String, String> mapElement = hListViewAdapter.getItem(position);
                final Intent intent = new Intent();
                Log.d("hiwonder", "mapElement id := " + mapElement.get("item_id").toUpperCase());
                Log.d("hiwonder", "mapElement ip := " + mapElement.get("item_ip").toUpperCase());
//                if(mapElement.get("item_id").toUpperCase().contains("JETMAX"))
                {
                    intent.setClass(getBaseContext(), TagRecognition.class);
                    intent.putExtra("deviceid", mapElement.get("item_id"));
                    intent.putExtra("deviceip", mapElement.get("item_ip"));
                    startActivity(intent);
                }
            }
        });

        hListView.setOnItemLongClickListener(new AdapterView.OnItemLongClickListener() {
            @Override
            public boolean onItemLongClick(AdapterView<?> parent, View view, int position, long id) {
                Log.d("hh_aa1", "position : = " + position);
                final HashMap<String, String> mapElement = hListViewAdapter.getItem(position);
                if (mapElement.get("item_ip").equals("192.168.149.1")) {
                    Log.d("hh_aa1", "1 : = ");
                } else {
                    Log.d("hh_aa1", "2 : = " + mapElement.get("item_ip"));
                    WifiDeviceInfoDialog.createDialog(getFragmentManager(), mapElement.get("item_ip"), mapElement.get("item_id"));
                }
                return false;
            }
        });
    }

    @Override
    protected void onResume() {
        RefreshBtnAnimStart(true);
        hListViewAdapter.clear();
        _scanner.scanAll();
        searchingInfoText.setText(R.string.searching);
        super.onResume();
    }

    @Override
    protected void onRightBtnClick(View v)
    {
        RefreshBtnAnimStart(true);
        hListViewAdapter.clear();
        _scanner.scanAll();
        searchingInfoText.setText(R.string.searching);
    }

    @Override
    public void onBackPressed() {
        if (!confirm) {
            confirm = true;
            Toast.makeText(this, R.string.exit_remind, Toast.LENGTH_SHORT).show();
            Timer timer = new Timer();
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    confirm = false;
                }
            }, 2000);
        } else {
            super.onBackPressed();
            android.os.Process.killProcess(android.os.Process.myPid());
        }
    }
}