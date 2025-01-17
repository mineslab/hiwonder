package com.example.armpiprocontrol;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.view.WindowManager;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.FrameLayout;
import android.widget.ImageButton;

import com.example.armpiprocontrol.R;

public class TitleActivity extends Activity implements View.OnClickListener {

    private ImageButton leftBtn;
    private ImageButton rightBtn;
    private FrameLayout mContentLayout;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setupViews();
    }

    private void setupViews() {
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        super.setContentView(R.layout.activity_title);
        mContentLayout = (FrameLayout) findViewById(R.id.layout_content);
        leftBtn = (ImageButton) findViewById(R.id.left_button);
        rightBtn = (ImageButton) findViewById(R.id.right_button);
        leftBtn.setOnClickListener(this);
        rightBtn.setOnClickListener(this);
    }

    public void rightBtn(boolean v) {
        if (v) {
            rightBtn.performClick();
        }
    }


    public void showLeftBtn(boolean show) {
        if (leftBtn != null) {
            if (show) {
                leftBtn.setVisibility(View.VISIBLE);
            } else {
                leftBtn.setVisibility(View.INVISIBLE);
            }
        }
    }

    public void showRightBtn(boolean show) {
        if (rightBtn != null) {
            if (show) {
                rightBtn.setVisibility(View.VISIBLE);
            } else {
                rightBtn.setVisibility(View.INVISIBLE);
            }
        }
    }
    /**
     * 返回按钮点击后触发
     *
     * @param backwardView
     */
    protected void onBackward(View backwardView) {
        // Toast.makeText(this, "点击返回，可在此处调用finish()", Toast.LENGTH_LONG).show();
        finish();
    }

    /**
     * 提交按钮点击后触发
     *
     * @param forwardView
     */
    protected void onRightBtnClick(View forwardView) {

    }

    public void RefreshBtnAnimStart(boolean flag) {
        if (flag) {
            final Animation setAnim = AnimationUtils.loadAnimation(this, R.anim.refresh_anim);
            rightBtn.startAnimation(setAnim);
        } else {
            rightBtn.clearAnimation();
        }
    }

    @Override
    public void setContentView(int layoutResID) {
        mContentLayout.removeAllViews();
        View.inflate(this, layoutResID, mContentLayout);
        onContentChanged();
    }

    @Override
    public void setContentView(View view) {
        mContentLayout.removeAllViews();
        mContentLayout.addView(view);
        onContentChanged();
    }

    @Override
    public void setContentView(View view, ViewGroup.LayoutParams params) {
        mContentLayout.removeAllViews();
        mContentLayout.addView(view, params);
        onContentChanged();
    }

    @Override
    public void onClick(View v) {
        int id = v.getId();
        if (id == R.id.left_button) {
            onBackward(v);
        } else if (id == R.id.right_button) {
            onRightBtnClick(v);
        }
    }
}