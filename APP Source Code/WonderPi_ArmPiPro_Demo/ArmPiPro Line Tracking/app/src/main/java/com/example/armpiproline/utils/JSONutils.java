package com.example.armpiproline.utils;

import android.util.Log;

import com.google.gson.Gson;

import org.json.JSONObject;

/**
 * created by hiwonder
 * on 2021/1/28
 * Explanation :websocket发送的json数据格式模板；
 */
public class JSONutils {
    public static String JsonData;

    /**
     * 玩法进入/退出
     * 例：enter / exit
     * {
     * "op": "call_service",
     * "service": "object_tracking/enter"
     * }
     */
    public static String send(String op, String service) {
        RequstBundle bundle = new RequstBundle();
        bundle.op = op;
        bundle.service = service;
        RequstBundle.RequstContent content = new RequstBundle.RequstContent();
        Log.e("wx", new Gson().toJson(bundle));
        return new Gson().toJson(bundle);
    }


    /**
     * 玩法开启/关闭/发送指令
     * 例：true / false   或参数值
     * {
     * "op": "call_service",
     * "service": "object_tracking/set_running",    物品追踪
     * "args":{data:true}
     * }
     */
    public static String send_data(String op, String service, String data) {
        JsonData = "{\"op\":\"" + op + "\",\"service\":\"/" + service + "\",\"args\":{\"data\":" + data + "}}";
        return JsonData;
    }

    public static String send_blue_color_sort(String op, String service, boolean sort) {
        String sortStr = "false";
        if(sort)
            sortStr = "true";
        JsonData = "{\"op\":\"" + op + "\",\"service\":\"/" + service + "\",\"args\":{\"color_name\":\"blue\"," + "\"is_enable\":" + sortStr + "}}";
        return JsonData;
    }

    public static class RequstBundle {
        private String op;
        private String service;
        private RequstContent args;

        public String getOp() {
            return op;
        }

        public void setOp(String op) {
            this.op = op;
        }

        public String getService() {
            return service;
        }

        public void setService(String service) {
            this.service = service;
        }

        public RequstContent getArgs() {
            return args;
        }

        public void setArgs(RequstContent args) {
            this.args = args;
        }

        public static class RequstContent {
            public String data;

            public String getData() {
                return data;
            }

            public void setData(String data) {
                this.data = data;
            }
        }
    }

    public static String modeEnter(String service)
    {
        String value = "";
        service = "/" + service + "/enter";
        try {
            JSONObject cmdInfo = new JSONObject();
            cmdInfo.put("op", "call_service");
            cmdInfo.put("service", service);
            JSONObject argInfo = new JSONObject();
            cmdInfo.put("args", argInfo);
            System.out.println(cmdInfo.toString());
            value = cmdInfo.toString();
        } catch (Exception ex) {

        }
        return value;
    }

    public static String modeExit(String service)
    {
        String value = "";
        service = "/" + service + "/exit";
        try {
            JSONObject cmdInfo = new JSONObject();
            cmdInfo.put("op", "call_service");
            cmdInfo.put("service", service);
            JSONObject argInfo = new JSONObject();
            cmdInfo.put("args", argInfo);
            System.out.println(cmdInfo.toString());
            value = cmdInfo.toString();
        } catch (Exception ex) {

        }
        return value;
    }
    public static String sendHeartBeat(String service)
    {
        String value = "";
        service = "/" + service + "/heartbeat";
        try {
            JSONObject cmdInfo = new JSONObject();
            cmdInfo.put("op", "call_service");
            cmdInfo.put("service", service);
            JSONObject argInfo = new JSONObject();
            argInfo.put("data", true);
            cmdInfo.put("args", argInfo);
            System.out.println(cmdInfo.toString());
            value = cmdInfo.toString();
        } catch (Exception ex) {

        }
        return value;
    }

    public static String sendRunning(String service, boolean run)
    {
        String value = "";
        service = "/" + service + "/set_running";
        try {
            JSONObject cmdInfo = new JSONObject();
            cmdInfo.put("op", "call_service");
            cmdInfo.put("service", service);
            JSONObject argInfo = new JSONObject();
            if (run)
                argInfo.put("data", true);
            else
                argInfo.put("data", false);
            cmdInfo.put("args", argInfo);
            System.out.println(cmdInfo.toString());
            value = cmdInfo.toString();
        } catch (Exception ex) {

        }
        return value;
    }
}

