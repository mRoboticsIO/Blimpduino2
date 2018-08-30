package com.jjrobots.blimpduino_v1;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.Intent;
import android.graphics.Canvas;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Vibrator;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;
import android.widget.Toast;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.Timer;
import java.util.TimerTask;

public class control extends AppCompatActivity {

    SeekBar SBthrottle;
    SeekBar SBsteering;
    Button Bup;
    Button Bdown;
    TextView textDebug;
    ToggleButton AltHold;
    int Bup_pressed=0;
    int Bdown_pressed=0;

    InetAddress address1;
    int puerto = 2222;

    public int iCH1=0;
    public int iCH2=0;
    public int iCH3=0;
    public int iCH4=0;
    public int iCH5=0;
    public int iCH6=0;
    public int iCH7=0;
    public int iCH8=0;

    public static Vibrator touchVibrator;

    public sendUDPmessage UDPM;

    public Thread UDPM_thread;
    public UDP_Server Server;

    volatile boolean activityStopped = false;
    Timer timer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // Remove title bar
        try
        {
            this.getSupportActionBar().hide();
        }
        catch (NullPointerException e){}
        setContentView(R.layout.activity_control);

        try {
            address1 = InetAddress.getByName("192.168.4.1");
        } catch (UnknownHostException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        SBthrottle = (SeekBar) findViewById(R.id.throttle);
        SBsteering = (SeekBar) findViewById(R.id.steering);
        Bup = (Button) findViewById(R.id.buttonUp);
        Bdown = (Button) findViewById(R.id.buttonDown);
        AltHold = (ToggleButton) findViewById(R.id.toggleButAlt);

        SBthrottle.setOnSeekBarChangeListener(new speedBarAxisListener());
        SBsteering.setOnSeekBarChangeListener(new speedBarAxisListener());
        Bup.setOnTouchListener(new buttonUpListener());
        Bdown.setOnTouchListener(new buttonDownListener());
        textDebug = (TextView)findViewById(R.id.textDebug);
        AltHold.setOnCheckedChangeListener(new test());

        touchVibrator = (Vibrator) getSystemService(Context.VIBRATOR_SERVICE);

        // Start UDP message sender
        UDPM = new sendUDPmessage();
        UDPM.iState = 0;  // Manual control mode
        UDPM_thread = new Thread(UDPM);
        UDPM_thread.start();

        // Global variables to store angle and battery values
        MyProperties.getInstance().angle = 0;
        MyProperties.getInstance().battery = 0;
        MyProperties.getInstance().debug = "Welcome!";

        //start UDP server
        Server = new UDP_Server();
        Server.runUdpServer();

        // Update debug text
        Thread t = new Thread() {

            @Override
            public void run() {
                try {
                    while (!isInterrupted()) {
                        Thread.sleep(500);
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                // update TextView here!
                                textDebug.setText(MyProperties.getInstance().debug);
                            }
                        });
                    }
                } catch (InterruptedException e) {
                }
            }
        };

        t.start();
    }

    @Override
    public void onPause()
    {
        super.onPause();
        activityStopped = true;
    }

    @Override
    public void onStop()
    {
        super.onStop();
        activityStopped = true;
    }

    @Override
    public void onResume()
    {
        super.onResume();
        activityStopped = false;
    }

    public class UDP_Server{
        private AsyncTask<Void, Void, Void> async;
        private boolean Server_aktiv = true;

        @SuppressLint("NewApi")
        public void runUdpServer(){
            async = new AsyncTask<Void, Void, Void>(){
                @Override
                protected Void doInBackground(Void... params){
                    byte[] lMsg = new byte[4096];
                    DatagramPacket dp = new DatagramPacket(lMsg, lMsg.length);
                    DatagramSocket ds = null;
                    //ByteBuffer ibuf = ByteBuffer.allocate(4);
                    Log.d("UDP","Start UDP SERVER");
                    try{
                        ds = new DatagramSocket(2223); //2223
                        //ds = new DatagramSocket(2223, InetAddress.getByName("0.0.0.0"));
                        //ds.setBroadcast(true);
                        Log.d("UDP","Start UDP SERVER port:2222");

                        while(Server_aktiv){
                            ds.receive(dp);
                            Log.d("UDP","Recibido");
                            // Check for packets starting with $ character
                            int index = 0;
                            if (dp.getLength()>8){
                                String debug = new String(lMsg,0,26);
                                //textDebug.setText(lMsg.toString());
                                Log.d("UDP",debug);
                                MyProperties.getInstance().debug = debug;
                            }
                            //Log.d("UDPR","ANGLE:" + (int) lMsg[0]+" "+(int)lMsg[2]);
                            //Log.d("UDPR","MESS2 "+dp.getLength()+" "+lMsg[0]+" A"+MyProperties.getInstance().angle);
                            //for (byte b : dp.getData()) {
                            //    if (b == (byte)0x24) {  // $ delimiter
                            //    Log.d("UDPR","DELIMITER");
                            //    }
                            //}
                        }
                        //Log.d("UDPR2",lMsg)
                    }
                    catch (Exception e){
                        e.printStackTrace();
                    }
                    finally{
                        if (ds != null){
                            ds.close();
                        }
                    }
                    return null;
                }
            };

            if (Build.VERSION.SDK_INT >= 11) async.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
            else async.execute();
        }

        public void stop_UDP_Server(){
            Server_aktiv = false;
        }
    }


    public class sendUDPmessage implements Runnable {

        public int iState;
        public void run() {
            byte[] buf = new byte[25];
            ByteBuffer ibuf = ByteBuffer.allocate(4);

            while (!activityStopped) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                if (iState == 0)
                    continue;

                iCH1 = (SBthrottle.getProgress() - 500);
                iCH2 = (SBsteering.getProgress() - 500);
                iCH3 = 0;
                if (Bup_pressed == 1)
                    iCH3 = 20;
                if (Bdown_pressed == 1)
                    iCH3 = -20;

                // Construct message (20 bytes: 4 bytes header + 4 ints)
                buf = "JJBA0000000000000000".getBytes();  // JJ Arm Manual control

                ibuf.putShort((short) iCH1);
                System.arraycopy(ibuf.array(), 0, buf, 4, 2);
                ibuf.clear();
                ibuf.putShort((short) iCH2);
                System.arraycopy(ibuf.array(), 0, buf, 6, 2);
                ibuf.clear();
                ibuf.putShort((short) iCH3);
                System.arraycopy(ibuf.array(), 0, buf, 8, 2);
                ibuf.clear();
                ibuf.putShort((short) iCH4);
                System.arraycopy(ibuf.array(), 0, buf, 10, 2);
                ibuf.clear();
                ibuf.putShort((short) iCH5);
                System.arraycopy(ibuf.array(), 0, buf, 12, 2);
                ibuf.clear();
                ibuf.putShort((short) iCH6);
                System.arraycopy(ibuf.array(), 0, buf, 14, 2);
                ibuf.clear();
                ibuf.putShort((short) iCH7);
                System.arraycopy(ibuf.array(), 0, buf, 16, 2);
                ibuf.clear();
                ibuf.putShort((short) iCH8);
                System.arraycopy(ibuf.array(), 0, buf, 18, 2);
                ibuf.clear();
                String str1 = new String(buf);
                Log.d("DMSG", "JJBA:"+String.valueOf(iCH1)+";"+String.valueOf(iCH2)+";"+String.valueOf(iCH3));  // DEBUG message
                //Log.d("DMSG", String.valueOf(iCH6));
                try {
                    DatagramSocket socket;
                    socket = new DatagramSocket();
                    socket.connect(address1, puerto);
                    socket.send(new DatagramPacket(buf, 20, address1, puerto));
                    socket.close();
                } catch (SocketException e1) {
                    e1.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            } // while (1)
        }
    }

    private class buttonUpListener implements Button.OnTouchListener{
        public boolean onTouch(View v, MotionEvent e){
            if (e.getAction() == MotionEvent.ACTION_DOWN ){
                Bup.setText("--");
                Bup_pressed = 1;
                touchVibrator.vibrate(80);
                UDPM.iState = 1;

            }
            if (e.getAction() == MotionEvent.ACTION_UP ){
                Bup.setText("UP");
                Bup_pressed = 0;
            }
            return true;
        }
    }
    private class buttonDownListener implements Button.OnTouchListener{
        public boolean onTouch(View v, MotionEvent e){
            if (e.getAction() == MotionEvent.ACTION_DOWN ){
                Bdown.setText("--");
                Bdown_pressed = 1;
                touchVibrator.vibrate(80);
                UDPM.iState = 1;
            }
            if (e.getAction() == MotionEvent.ACTION_UP ){
                Bdown.setText("DOWN");
                Bdown_pressed = 0;
            }
            return true;
        }
    }

    private class test implements ToggleButton.OnCheckedChangeListener{
        @Override
        public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            touchVibrator.vibrate(80);
            if (isChecked) {
                iCH5=1;
                UDPM.iState = 1; // Seems to activate the UDP????? Jordi.
            } else {
                // The toggle is disabled
                iCH5=0;
            }
        }
    }


    private class speedBarAxisListener implements SeekBar.OnSeekBarChangeListener {

        public void onProgressChanged(SeekBar seekBar, int progress,
                                      boolean fromUser) {
            // Log the progress
            //Log.d("DP", "Progress: " + progress);

            textDebug.setText(MyProperties.getInstance().debug);

            UDPM.iState = 1;
            // SOCKET message should be implemented on other thread
            //sendUDPmessage UDPM = new sendUDPmessage();
            //UDPM.iState = 0;  // Manual control mode
            //new Thread(UDPM).start();
            //TextDebug.setText("1:"+String.valueOf((float)iCH1/100.0)+" 2:"+String.valueOf((float)iCH2/100.0)+ " 3:"+String.valueOf((float)iCH3/100.0));
        }

        public void onStartTrackingTouch(SeekBar seekBar) {
            touchVibrator.vibrate(80);
        }

        public void onStopTrackingTouch(SeekBar seekBar) {
            touchVibrator.vibrate(80);
            seekBar.setProgress(500);
        }

    }

    public void buttonUpOnClick(View v) {


    }


}
