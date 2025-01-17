package com.example.armpiprotag.dialog;

import android.app.DialogFragment;
import android.app.FragmentManager;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import com.example.armpiprotag.R;

/**
 * Created by andy on 2017/9/29.
 */

public class WifiDeviceInfoDialog extends DialogFragment implements View.OnClickListener {
    private String ip;
    private String id;
    private Button okBtn;

    private TextView ipTx;
    private TextView idTx;

    private ImageView deviceIcon;

    public static void createDialog(FragmentManager fragmentManager, String ip, String id) {
        WifiDeviceInfoDialog dialog = new WifiDeviceInfoDialog();
        dialog.ip = ip;
        dialog.id = id;
        dialog.show(fragmentManager, "searchDialog");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setStyle(STYLE_NO_TITLE, getTheme());
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        return inflater.inflate(R.layout.layout_deviceinfo, container, false);

    }

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        deviceIcon = (ImageView) view.findViewById(R.id.img_list_item);
        ipTx = (TextView) view.findViewById(R.id.device_ip);
        idTx = (TextView) view.findViewById(R.id.device_id);
        okBtn = (Button) view.findViewById(R.id.dialog_btn);
        okBtn.setOnClickListener(this);
        ipTx.setText(ip);
        deviceIcon.setImageResource(R.drawable.armpipro);
        int index = id.indexOf(":");
        String idStr = id.substring(index + 1);
        idTx.setText(idStr);
    }

    public void onDestroyView() {
        super.onDestroyView();
    }

    @Override
    public void onClick(View v) {
        dismissAllowingStateLoss();
    }
}
