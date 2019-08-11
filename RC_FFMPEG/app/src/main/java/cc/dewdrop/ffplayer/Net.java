package cc.dewdrop.ffplayer;

import android.content.Context;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Collections;
import java.util.Enumeration;
import java.util.List;




public class Net {
    //private static final int MAX_UDP_DATAGRAM_LEN = 1500;


    /**
     * Get IP address from first non-localhost interface
     *
     * @param useIPv4 true=return ipv4, false=return ipv6
     * @return address or empty string
     */
    public static String getIPAddress(boolean useIPv4) {
        try {
            List<NetworkInterface> interfaces = Collections.list(NetworkInterface.getNetworkInterfaces());
            for (NetworkInterface intf : interfaces) {
                List<InetAddress> addrs = Collections.list(intf.getInetAddresses());
                for (InetAddress addr : addrs) {
                    if (!addr.isLoopbackAddress()) {
                        String sAddr = addr.getHostAddress();
                        //boolean isIPv4 = InetAddressUtils.isIPv4Address(sAddr);
                        boolean isIPv4 = sAddr.indexOf(':') < 0;

                        if (useIPv4) {
                            if (isIPv4)
                                return sAddr;
                        } else {
                            if (!isIPv4) {
                                int delim = sAddr.indexOf('%'); // drop ip6 zone suffix
                                return delim < 0 ? sAddr.toUpperCase() : sAddr.substring(0, delim).toUpperCase();
                            }
                        }
                    }
                }
            }
        } catch (Exception ignored) {
        } // for now eat exceptions
        return "";
    }


    public static boolean net_runing = true;
    private static int SERVER_PORT = 9876;
    static private boolean ip_OK = false;

    public static String getIpAddress() {
        try {
            for (Enumeration en = NetworkInterface.getNetworkInterfaces(); en.hasMoreElements(); ) {
                NetworkInterface intf = (NetworkInterface) en.nextElement();
                for (Enumeration enumIpAddr = intf.getInetAddresses(); enumIpAddr.hasMoreElements(); ) {
                    InetAddress inetAddress = (InetAddress) enumIpAddr.nextElement();
                    if (!inetAddress.isLoopbackAddress() && inetAddress instanceof Inet4Address) {
                        String ipAddress = inetAddress.getHostAddress().toString();
                        Log.e("IP address", "" + ipAddress);
                        return ipAddress;
                    }
                }
            }
        } catch (SocketException ex) {
            Log.e("Socket exception", ex.toString());
        }
        return null;
    }


    public Net(int port, int timeOut) {
        SERVER_PORT = port;
    }

    public void stop() {

        //udp_server_run=false;
        //client_runing=false;
    }

    static InetAddress copterAddress, myIP = null;
    int port = 0;

    static int threads = 0;

    static Context context;

    public void start() {
        Log.d("NET", "start");
        if (threads > 0 || net_runing == false) {
            Log.d("NET", "exit");
            return;
        }
        threads++;
        Thread thread = new Thread() {
            @Override
            public void run() {
                //		Disk.close();
                //		Disk.createOrOpen();
                while (net_runing) {

                    //connect2esp();
                    String myIP = getIpAddress();
                    //    String myIP=getIPAddress(true);


                    // String serverIPandPort[]={"192.168.1.112:9876","192.168.1.112:9876"};//Disk.getIP(myIP);
                    String serverIPandPort[] = Disk.getIP(myIP);


                    int i = 0;
                    while (serverIPandPort[i] != null && serverIPandPort[i].length() > 10) {

                        Log.d("UDP", "CONNECT TO " + Integer.toString(i));
                        if (runTCPClient(serverIPandPort[i]) == true) {
                            if (ip_OK == false) {
                                i++;
                                if (i >= serverIPandPort.length)
                                    i = 0;
                            }
                        }

                    }
                }
            }
        };
        thread.start();
    }

    final int buf_size=16384;
    byte buffer[] = new byte[buf_size];

    final int IP = 0;
    final int PORT = 1;
    boolean dataloaded = false;
    static int threads_=0;

    public boolean runTCPClient(String ip_port) {
        if (threads_>0)
            return false;
        boolean run =true;
        threads_++;
        DatagramSocket udpSocket=null;
        if (dataloaded == false) {
            dataloaded = true;
            Disk.loadLatLonAlt("/sdcard/RC/lostCon_location.save", true);
        }
        if (ip_port == null)
            return false;
        String s[] = ip_port.split(":");
        try {

            ///////////
            try {
                copterAddress = InetAddress.getByName(s[IP]);
                SERVER_PORT = Integer.parseInt(s[PORT]);

                udpSocket = new DatagramSocket(SERVER_PORT);
                InetAddress serverAddr = copterAddress;
                int offline_cnt=0;
                while (net_runing && run) {
                    try {
                        int len = Commander.get(buffer);
                        DatagramPacket packet = new DatagramPacket(buffer, len, serverAddr, SERVER_PORT);
                       // Log.d("UDP","send data. threads="+threads_);
                        udpSocket.send(packet);
                        DatagramPacket packet1 = new DatagramPacket(buffer, buf_size);
                       // Log.d("UDP","about to wait to receive. threads="+threads_);
                       udpSocket.setSoTimeout(100);
                        udpSocket.receive(packet1);
                        Commander.link = true;
                        ip_OK = true;
                        len=packet1.getLength();
                        Telemetry.bufferReader_(buffer, len);
                        offline_cnt=0;
                            //  Log.d("Receivedtext", text);

                        // try {Thread.sleep(10);} catch (InterruptedException e) {e.printStackTrace(); }

                    } catch (IOException e) {
                        if (offline_cnt++>3) {
                            run=false;
                            if (Commander.link)
                                Disk.saveLatLonAlt("/sdcard/RC/lostCon_location.save", Telemetry.lat, Telemetry.lon, Telemetry._alt);
                            Commander.link = false;
                            Log.d("UDP", "Error1:");
                        }
                    }

                }
            }catch(IOException e) {
                Log.d("UDP", "Error00000:", e);
            }

        } finally {
            threads_--;
            Log.d("UDP", "Finally:");
            if (udpSocket!=null)
                udpSocket.close();
            if (Commander.link)
                Disk.saveLatLonAlt("/sdcard/RC/lostCon_location.save", Telemetry.lat, Telemetry.lon, Telemetry._alt);
            Commander.link = false;
            if (MainActivity.drawView != null)
                MainActivity.drawView.postInvalidate();
            Log.i("UDP", "UDP_CLIENT KILLED!");


        }
        return false;
    }
}

