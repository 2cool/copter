package cc.dewdrop.ffplayer;

import android.util.Log;

import java.io.IOException;
import java.io.InterruptedIOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;

public class UDPReceiveServer implements Runnable {

    private int receivePort;
    RecievedMessageListener messageListener;
    static int SOCKET_TIMEOUT = 1000;

    public UDPReceiveServer() {

    }

    public void setReceivePort(int receivePort) {
        this.receivePort = receivePort;
    }
    final int buf_size=1024;

    byte bufferout[] = new byte[buf_size];
    public void run() {



        DatagramSocket receiveSocket;
        //Log.d("CommandServer","Starting UDP receive server with port: " +Integer.toString(receivePort));
        try {
            while(!Thread.currentThread().isInterrupted() && Net.net_runing) {
                receiveSocket = new DatagramSocket(receivePort);
                receiveSocket.setSoTimeout(SOCKET_TIMEOUT);
                while (!Thread.currentThread().isInterrupted() && Net.net_runing) {
                    try {
                        int len = receiveCommandPacket(receiveSocket);
                    } catch (IOException e) {
                        Log.d("CommandServer","1");
                        Net.run = false;
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
                            Log.d("CommandServer","2");
                        }
                    }

                }
                Log.d("CommandServer","4");
                receiveSocket.close();
            }

        }
        catch (Exception e) {
            Log.d("CommandServer","5");
            e.printStackTrace();
        }
    }

    private int receiveCommandPacket(DatagramSocket socket) throws IOException {

        DatagramPacket receivePacket = new DatagramPacket(bufferout, buf_size);
        socket.receive(receivePacket);

        Commander.link = true;

        Net.ip_OK = true;
        int len = receivePacket.getLength();
        Telemetry.bufferReader_(bufferout, len);
        return len;

    }

    public interface RecievedMessageListener {
        void onRecieveMessage (String message);
    }


}
