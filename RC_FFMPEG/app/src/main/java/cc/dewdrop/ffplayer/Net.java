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
    static public boolean ip_OK = false;

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



    static int threads = 0;

    static Context context;

    public void start() {
       // if (true)
       //     return;
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

    final int buf_size=1024;
    byte buffer[] = new byte[buf_size];
    byte bufferout[] = new byte[buf_size];
    final int IP = 0;
    final int PORT = 1;

    static int threads_=0;
    private long  old_t=0;

    int errors_cnt=0;







    private UDPSendServer sendServer;
    private Thread sendThread;
    private UDPReceiveServer receiveServer;
    private Thread receiveThread;



    static public boolean run =true;
    public boolean runTCPClient(String ip_port) {
        if (threads_>0)
            return false;

        threads_++;

/*
        if (ip_port == null)
            return false;
        String s[] = ip_port.split(":");

        SERVER_PORT = Integer.parseInt(s[PORT]);


        sendServer = new UDPSendServer("send server");
        sendServer.setDestination(s[IP], SERVER_PORT);
        sendServer.setUpdateRate(60);

        receiveServer = new UDPReceiveServer();
        receiveServer.setReceivePort(SERVER_PORT+1);

        sendThread = new Thread(sendServer);
        receiveThread = new Thread(receiveServer);
        sendThread.start();
      //  receiveThread.start();

*/





        String controller_ip = "192.168.1.177";
        int sendPort = 9876;
        int recievePort = 9877;
        int updateRate = 60;


        sendServer = new UDPSendServer("send server");
        sendServer.setDestination(controller_ip, sendPort);
        sendServer.setUpdateRate(updateRate);

        receiveServer = new UDPReceiveServer();
        receiveServer.setReceivePort(recievePort);

        sendThread = new Thread(sendServer);
        receiveThread = new Thread(receiveServer);
        sendThread.start();
        receiveThread.start();











        while (net_runing) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

/*
        try {

            ///////////






                while (net_runing && run) {
                    try {
                        {
                            DatagramSocket udpSocket = new DatagramSocket(SERVER_PORT);
                           // udpSocket.setReuseAddress(true);
                            udpSocket.setSoTimeout(500);
                            InetAddress copterAddress = InetAddress.getByName(s[IP]);
                            // Log.d("UDP", "copter address: "+copterAddress);
                          //  while (net_runing && run) {
                                int len = Commander.get_b(buffer);
                                //Log.d("UDP", "len="+len);
                                DatagramPacket packet = new DatagramPacket(buffer, len, copterAddress, SERVER_PORT);
                                //  udpSocket.connect(copterAddress, SERVER_PORT);
                                udpSocket.send(packet);
                                //  Log.d("UDP", "ok=");
                                DatagramPacket packet1 = new DatagramPacket(bufferout, buf_size);
                              //  udpSocket.receive(packet1);

                                Commander.link = true;
                                ip_OK = true;
                                len = packet1.getLength();
                                //  Log.d("UDP", "recived : "+len);
                                Telemetry.bufferReader_(bufferout, len);
                           // }
                            udpSocket.close();

                        }

                    } catch (IOException e) {
                        errors_cnt++;
                        Log.d("UDP", "ERROR: "+errors_cnt+" "+e.getMessage());
                            run=false;
                            if (Commander.link) {
                                Telemetry.hom_pos_is_loaded = false;

                                Disk.save_location_(
                                        "/sdcard/RC/lostCon_location.save",
                                        Telemetry.lat,
                                        Telemetry.lon,
                                        Telemetry._alt);
                                Commander.link = false;

                                try {
                                    Thread.sleep(100);
                                } catch (InterruptedException ex) {
                                    ex.printStackTrace();
                                }

                                /// Log.d("UDP", "Error1:");
                            }
                    }
                }


        } finally {
            threads_--;
            Log.d("UDP", "Finally:");
          //  if (udpSocket!=null)
           //     udpSocket.close();
          //  if (Commander.link)

            Commander.link = false;
            if (MainActivity.drawView != null)
                MainActivity.drawView.postInvalidate();
            Log.i("UDP", "UDP_CLIENT KILLED!");


        }
 */

        return false;
    }
}

