package com.example.armpiprocontrol.component;

/**
 * Created by andy on 2017/9/19.
 */

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.drawable.Drawable;
import android.media.ThumbnailUtils;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.ImageView;
import android.widget.TextView;
import com.example.armpiprocontrol.R;
import java.util.ArrayList;
import java.util.HashMap;



public class HorizontalListViewAdapter extends BaseAdapter {
    private Context mContext;
    private LayoutInflater mInflater;
    Bitmap iconBitmap;
    Bitmap iconBitmap_bg;
    private int selectIndex = -1;
    private ArrayList<HashMap<String, String>> list;

    public HorizontalListViewAdapter(Context context) {
        this.mContext = context;
        this.list = new ArrayList<HashMap<String, String>>();
        mInflater = (LayoutInflater) mContext.getSystemService(Context.LAYOUT_INFLATER_SERVICE);//LayoutInflater.from(mContext);
    }

    public void add(HashMap<String, String> deviceInfo) {
        list.add(deviceInfo);
        notifyDataSetChanged();
    }

    public void remove(int pos) {
        list.remove(pos);
        notifyDataSetChanged();
    }

    public void clear() {
        list.clear();
        notifyDataSetChanged();
    }

    @Override
    public int getCount() {
        return list.size();
    }

    @Override
    public HashMap<String, String> getItem(int position) {
        return list.get(position);
    }

    @Override
    public long getItemId(int position) {
        return position;
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent) {

        ViewHolder holder;
        if (convertView == null) {
            holder = new ViewHolder();
            convertView = mInflater.inflate(R.layout.horizontal_list_layout, null);
            holder.mImage = (ImageView) convertView.findViewById(R.id.img_list_item);
            holder.mTitle = (TextView) convertView.findViewById(R.id.text_list_item);
            convertView.setTag(holder);
        } else {
            holder = (ViewHolder) convertView.getTag();
        }
        if (position == selectIndex) {
            convertView.setSelected(true);
        } else {
            convertView.setSelected(false);
        }
        HashMap<String, String> deviceInfo = list.get(position);
        String itemId = deviceInfo.get("item_id");

        holder.mTitle.setText(itemId);
        Log.d("hh_id", "id  : = " + itemId);
        iconBitmap = getPropThumnail(R.drawable.armpipro);
        holder.mImage.setImageBitmap(iconBitmap);
        return convertView;
    }

    private static class ViewHolder {
        private TextView mTitle;
        private ImageView mImage;
    }

    private Bitmap getPropThumnail(int id) {
        Drawable d = mContext.getResources().getDrawable(id);
        Bitmap b = BitmapUtil.drawableToBitmap(d);
        int w = mContext.getResources().getDimensionPixelOffset(R.dimen.thumnail_default_width);
        int h = mContext.getResources().getDimensionPixelSize(R.dimen.thumnail_default_height);
        Bitmap thumBitmap = ThumbnailUtils.extractThumbnail(b, w, h);

        return thumBitmap;
    }

    public void setSelectIndex(int i) {
        selectIndex = i;
    }
}
