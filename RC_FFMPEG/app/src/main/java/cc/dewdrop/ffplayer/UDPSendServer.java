package cc.dewdrop.ffplayer;

import android.util.Log;
import android.widget.Toast;

import java.io.IOException;
import java.io.InterruptedIOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Map;
import android.content.Context;


/**
 * Created by cameronfranz on 7/13/16.
 */
public class UDPSendServer implements Runnable {

    private String ctrl_state_outgoing;
    private String destinationIP;
    private int destinationPort;
    private int updateRate;


    public UDPSendServer(String outgoingMap) {
        ctrl_state_outgoing = outgoingMap;
    }

    public void setDestination(String ip,int port) {
        destinationIP = ip;
        destinationPort = port;
    }

    public void setUpdateRate (int rate) {
        updateRate = rate;
    }


    final int buf_size=1024;
    byte buffer[] = new byte[buf_size];

    public void run() {
        DatagramSocket sendSocket;
        //Log.d("CommandServer","Starting UDP send server with IP destination: " + destinationIP +
        //        " and port: " + Integer.toString(destinationPort));
        try {
            sendSocket = new DatagramSocket();
            while(!Thread.currentThread().isInterrupted() && Net.net_runing) {
                try {
                  //  String stringOut = ctrl_state_outgoing.toString().replace("=", ":");

                    int len = Commander.get_b(buffer);
                    sendCommandPacket(sendSocket, len);
                    Log.d("CommandServer","sended");
                }
                catch (UnknownHostException e) {
                    Log.d("CommandServer","Invalid destination IP"+e.getMessage());
                    //e.printStackTrace();
                }
                catch (IOException e) {
                    Log.d("CommandServer","Error sending packet");
                    //e.printStackTrace();
                }
                try {
                    Thread.sleep(100);
                }
                catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            Log.d("CommandServer","Shutting down UDP send server...");
            sendSocket.close();
        }
        catch (Exception e) {
            Log.d("CommandServer","Error binding send socket");
            //e.printStackTrace();
        }
    }

    private void sendCommandPacket (DatagramSocket socket,  int len) throws IOException {
        DatagramPacket sendPacket = new DatagramPacket(buffer, len,InetAddress.getByName(destinationIP),destinationPort);
        socket.send(sendPacket);
        //Log.d("CommandThread","Sent message: " + stringOut);
    }



}

