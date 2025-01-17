package com.example.armpiproline.connect;

/**
 * Created by andy on 2017/9/18.
 */

import android.content.Context;
import android.net.DhcpInfo;
import android.net.wifi.WifiManager;
import android.net.wifi.WifiManager.MulticastLock;
import android.os.AsyncTask;

import java.io.IOException;
import java.math.BigInteger;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

public class Scanner {
    private Context _context;
    private Scanner.OnScanOverListener _onScanOverListener;
    private boolean _getFirst;
    private boolean _scanning = false;
    private WifiManager _wifiManager;
    private MulticastLock _multicastLock;

    public Scanner(Context context) {
        this._context = context;
        this._wifiManager = (WifiManager) this._context.getSystemService(Context.WIFI_SERVICE);
    }

    private boolean scan(boolean getFirst) {
        if (this._scanning) {
            return false;
        } else {
            this._scanning = true;
            this._getFirst = getFirst;
            this._multicastLock = this._wifiManager.createMulticastLock("UDPwifi");
            this._multicastLock.acquire();
            Scanner.ScanAsyncTask scanAsyncTask = new Scanner.ScanAsyncTask();
            scanAsyncTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, new Void[0]);
            return true;
        }
    }

    private InetAddress getGatewayAddress() {
        DhcpInfo dhcp = this._wifiManager.getDhcpInfo();

        byte[] ip = BigInteger.valueOf((long) dhcp.serverAddress).toByteArray();
        InetAddress gatewayAddress = null;

        try {
            gatewayAddress = InetAddress.getByAddress(ip);
        } catch (UnknownHostException var5) {

        }
        return gatewayAddress;
    }


    private byte[] copyOfRange(byte[] from, int start) {
        int index = 0;

        for (int i = 0; i < from.length; i++) {
            if (from[i] == 0x3A) {
                index = i;
            }
        }

        //  int length = index + 11;//设备id固定12个字符   visionPi:D0fded

        int length = index + 9;//设备id固定12个字符

        byte[] result = new byte[length];

        System.arraycopy(from, start, result, 0, length);
        return result;
    }

    public boolean scanAll() {
        return this.scan(false);
    }

    public boolean scan() {
        return this.scan(true);
    }

    public void setOnScanOverListener(Scanner.OnScanOverListener listener) {
        this._onScanOverListener = listener;
    }

    public interface OnScanOverListener {
        void onResult(Map<InetAddress, String> var1, InetAddress var2);
    }

    private class ScanAsyncTask extends AsyncTask<Void, Void, Map<InetAddress, String>> {
        private int deviceIdBeginIndex;

        private ScanAsyncTask() {
            this.deviceIdBeginIndex = 0;
        }

        protected Map<InetAddress, String> doInBackground(Void... voids) {
            Map<InetAddress, String> list = new HashMap();

            for (int j = 0; j < 10; ++j) {
                DatagramSocket socket = null;
                try {
                    if (socket == null) {
                        socket = new DatagramSocket(null);
                        socket.setReuseAddress(true);
                        socket.bind(new InetSocketAddress(9025));
                        socket.setSoTimeout(350);
                    }
                    String netAddress = "255.255.255.255";
                    String sendStr = "LOBOT_NET_DISCOVER";
                    byte[] bufSend = sendStr.getBytes();
                    InetAddress ipaddress = InetAddress.getByName(netAddress);
                    DatagramPacket datagramPacket = new DatagramPacket(bufSend, bufSend.length, ipaddress, 9027);

                    // 发送数据
                    socket.send(datagramPacket);
                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    for (int i = 0; i < 10; ++i) {
                        try {
                            byte[] buf = new byte[30];
                            DatagramPacket receivedPacket = new DatagramPacket(buf, buf.length);
                            socket.receive(receivedPacket);
                            InetAddress address = InetAddress.getByName(receivedPacket.getAddress().getHostAddress());
                            if (!list.containsKey(address) && receivedPacket.getLength() > this.deviceIdBeginIndex) {
                                String deviceId = new String(Scanner.this.copyOfRange(buf, this.deviceIdBeginIndex));
                                list.put(address, deviceId);
                                if (Scanner.this._getFirst) {
                                    return list;
                                }
                            }
                        } catch (SocketException var18) {
                            var18.printStackTrace();
                        }
                    }
                } catch (SocketException var19) {
                    var19.printStackTrace();
                } catch (IOException var20) {
                    var20.printStackTrace();
                } finally {
                    if (socket != null) {
                        socket.disconnect();
                    }
                }
            }
            return list;
        }

        protected void onPostExecute(Map<InetAddress, String> addresses) {
            super.onPostExecute(addresses);
            Scanner.this._multicastLock.release();
            if (Scanner.this._onScanOverListener != null) {
                Scanner.this._onScanOverListener.onResult(addresses, Scanner.this.getGatewayAddress());

            }
            Scanner.this._scanning = false;
            System.gc();
        }
    }
}
