package com.example.paulforster.nxtapp;

import android.bluetooth.BluetoothAdapter;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.PersistableBundle;
import android.support.annotation.NonNull;
import android.support.design.widget.BottomNavigationView;
import android.support.v4.app.FragmentTransaction;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MenuItem;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.Toast;
import java.util.Timer;
import java.util.TimerTask;


import de.amr.plt.rcParkingRobot.AndroidHmiPLT;
import parkingRobot.INxtHmi;

//TODO einzeichnen von Fremnden Gegenständen in die Karte
//TODO warum kann ich da nicht parken?
//TODO ParkIcons neben die Linie

public class MainActivity extends AppCompatActivity {

    static final int REQUEST_ENABLE_BT = 154;
    static AndroidHmiPLT hmiModule = null;
    static ArrayAdapter<String> BTArrayAdapter;
    static BluetoothDialogFragment btDialog = null;
    static Timer refreshTimer;
    static TimerTask refreshTimerTask;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main );
        //Kein Bluetooth --> keine App
        BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth ist auf ihrem Gerät nicht verfügbar! :(",
                    Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        BottomNavigationView bottomNavigationView = (BottomNavigationView)
                findViewById(R.id.bottom_navigation);
        for(int i=0; i<3; i++) {
            bottomNavigationView.getMenu().getItem(i).setChecked(false);
        }
        bottomNavigationView.setOnNavigationItemSelectedListener(
                new BottomNavigationView.OnNavigationItemSelectedListener() {
                    @Override
                    public boolean onNavigationItemSelected(@NonNull MenuItem item) {
                        switch (item.getItemId()) {
                            case R.id.mode_pause:
                                hmiModule.setMode(INxtHmi.Mode.PAUSE);
                                break;
                            case R.id.mode_scout:
                                hmiModule.setMode(INxtHmi.Mode.SCOUT);
                                break;
                            case R.id.mode_park:
                                hmiModule.setMode(INxtHmi.Mode.PARK_NOW);
                                Toast toast = Toast.makeText(getApplicationContext(),
                                        "Nächste Parklücke wird angefahren", Toast.LENGTH_LONG);
                                toast.show();
                                break;
                        }
                        return false;
                    }
                });
        getSupportFragmentManager().enableDebugLogging(true);
        btDialog = new BluetoothDialogFragment();
        MapFragment mapFragment = new MapFragment();
        ParkFragment parkFragment = new ParkFragment();
        FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
        ft.add(R.id.content, mapFragment, "MAPFRAGMENT");
        ft.add(R.id.content, parkFragment, "PARKFRAGMENT");
        ft.hide(parkFragment);
        ft.commit();
        /**
        if (savedInstanceState == null) {
            btDialog = new BluetoothDialogFragment();
            MapFragment mapFragment = new MapFragment();
            ParkFragment parkFragment = new ParkFragment();
            FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
            ft.add(R.id.content, mapFragment, "MAPFRAGMENT");
            ft.add(R.id.content, parkFragment, "PARKFRAGMENT");
            ft.hide(parkFragment);
            ft.commit();
            getSupportFragmentManager().executePendingTransactions();
        } else {
            MapFragment mapFragment = (MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT");
            ParkFragment parkFragment = (ParkFragment) getSupportFragmentManager().findFragmentByTag("PARKFRAGMENT");
        }
         */
}

    @Override
    public void onSaveInstanceState(Bundle outState, PersistableBundle outPersistentState) {
        super.onSaveInstanceState(outState, outPersistentState);
    }

    @Override
    protected void onStart() {
        super.onStart();

        Log.e("AConPause", "onStart");

        reScheduleTimer(100);
    }

    @Override
    protected void onPause() {
        super.onStop();
        Log.e("AConPause", "refreshTimer.cancel() happens");
        refreshTimer.cancel();
        refreshTimer = null;
    }

    @Override
    protected void onResume() {
        super.onResume();

        Log.e("AConPause", "onResume");
        //TODO wenn man gerade Bluetooth eingeschaltet hat, findet er die Liste noch nicht...
        if (!btDialog.isAdded()) if(hmiModule == null) btDialog.show(getSupportFragmentManager(), "btDialog");
        else if (!hmiModule.isConnected()) btDialog.show(getSupportFragmentManager(), "btDialog");
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if(requestCode == REQUEST_ENABLE_BT){
            if(resultCode != RESULT_OK){
                finish();
            }
        }
    }

    public void reScheduleTimer(int duration) {
        refreshTimer = new Timer();
        refreshTimerTask = new MyTimerTask();
        refreshTimer.schedule(refreshTimerTask, 0, duration);
    }

    private class MyTimerTask extends TimerTask {
        @Override
        public void run() {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    refreshMenu();
                }
            });
        }
    }

    public void refreshMenu(){
                    BottomNavigationView bottomNavigationView = (BottomNavigationView)
                            findViewById(R.id.bottom_navigation);
                    FragmentTransaction fragmentTransaction = getSupportFragmentManager().beginTransaction();
                    if (hmiModule != null)
                        if(hmiModule.isConnected()) {
                            Log.e("Status", "is verbunden");
                            if(hmiModule.getCurrentStatus() != null) {
                                Log.e("Status", "is da");
                                switch (hmiModule.getCurrentStatus()) {
                                    case SCOUT:
                                        bottomNavigationView.getMenu().getItem(0).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(1).setChecked(true);
                                        bottomNavigationView.getMenu().getItem(2).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(0).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(1).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(2).setEnabled(true);
                                        for(ImageView imageView : ((MapFragment)getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT")).imageviewArrayList){
                                            imageView.setClickable(true);
                                        }
                                        fragmentTransaction.hide((ParkFragment)getSupportFragmentManager().findFragmentByTag("PARKFRAGMENT"));
                                        fragmentTransaction.commit();
                                        if ((((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler != null){
                                            String data = "MSG DATA";
                                            Message msg = (((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler.obtainMessage(0, data);
                                            msg.sendToTarget();
                                        }
                                        break;
                                    case PARK_NOW:
                                        bottomNavigationView.getMenu().getItem(0).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(1).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(2).setChecked(true);
                                        bottomNavigationView.getMenu().getItem(0).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(1).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(2).setEnabled(true);
                                        for(ImageView imageView : ((MapFragment)getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT")).imageviewArrayList){
                                            imageView.setClickable(true);
                                        }
                                        if ((((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler != null){
                                            String data = "MSG DATA";
                                            Message msg = (((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler.obtainMessage(0, data);
                                            msg.sendToTarget();
                                        }
                                        fragmentTransaction.show((ParkFragment)getSupportFragmentManager().findFragmentByTag("PARKFRAGMENT"));
                                        fragmentTransaction.commit();
                                        /**
                                        if(getSupportFragmentManager().getBackStackEntryCount()==0){
                                            FragmentTransaction ft = getSupportFragmentManager()
                                                    .beginTransaction();
                                            ParkFragment parkFragment = new ParkFragment();
                                            ft.add(R.id.content, parkFragment);
                                            ft.addToBackStack(null);
                                            ft.commit();
                                        }*/
                                        break;
                                    case PARK_THIS:
                                        bottomNavigationView.getMenu().getItem(0).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(1).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(2).setChecked(true);
                                        bottomNavigationView.getMenu().getItem(0).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(1).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(2).setEnabled(true);
                                        for(ImageView imageView : ((MapFragment)getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT")).imageviewArrayList){
                                            imageView.setClickable(true);
                                        }
                                        fragmentTransaction.show((ParkFragment)getSupportFragmentManager().findFragmentByTag("PARKFRAGMENT"));
                                        fragmentTransaction.commit();
                                        /**
                                        if(getSupportFragmentManager().getBackStackEntryCount()==0){
                                            FragmentTransaction ft = getSupportFragmentManager()
                                                    .beginTransaction();
                                            ParkFragment parkFragment = new ParkFragment();
                                            ft.add(R.id.content, parkFragment);
                                            ft.addToBackStack(null);
                                            ft.commit();
                                        }
                                         */
                                        break;
                                    case PARKED:
                                        bottomNavigationView.getMenu().getItem(0).setChecked(true);
                                        bottomNavigationView.getMenu().getItem(1).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(2).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(0).setEnabled(false);
                                        bottomNavigationView.getMenu().getItem(1).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(2).setEnabled(false);

                                        for(ImageView imageView : ((MapFragment)getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT")).imageviewArrayList){
                                            imageView.setClickable(false);
                                        }
                                        fragmentTransaction.hide((ParkFragment)getSupportFragmentManager().findFragmentByTag("PARKFRAGMENT"));
                                        fragmentTransaction.commit();
                                        if ((((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler != null){
                                            String data = "MSG DATA";
                                            Message msg = (((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler.obtainMessage(0, data);
                                            msg.sendToTarget();
                                        }
                                        break;

                                    case INACTIVE:
                                        bottomNavigationView.getMenu().getItem(0).setChecked(true);
                                        bottomNavigationView.getMenu().getItem(1).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(2).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(0).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(1).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(2).setEnabled(true);
                                        //TODO die Arraylist mit den ParkIcons muss hier immernoch existieren
                                        for(ImageView imageView : ((MapFragment)getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT")).imageviewArrayList){
                                            imageView.setClickable(true);
                                        }
                                        fragmentTransaction.hide((ParkFragment)getSupportFragmentManager().findFragmentByTag("PARKFRAGMENT"));
                                        fragmentTransaction.commit();
                                        if ((((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler != null){
                                            String data = "MSG DATA";
                                            Message msg = (((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler.obtainMessage(0, data);
                                            msg.sendToTarget();
                                        }
                                        break;
                                }
                            }
                        } else {
                            for(int i = 0; i<3; i++){
                                bottomNavigationView.getMenu().getItem(i).setChecked(false);
                                bottomNavigationView.getMenu().getItem(i).setEnabled(false);
                                getSupportFragmentManager().popBackStack();
                            }
                        }
                }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
        if (hmiModule != null && hmiModule.connected) {

            terminateBluetoothConnection();
        }
    }

    /**
     * Terminate the bluetooth connection to NXT
     */
    private void terminateBluetoothConnection(){
        Toast.makeText(this, "Bluetooth connection was terminated!", Toast.LENGTH_LONG).show();
        hmiModule.setMode(INxtHmi.Mode.PAUSE);
        hmiModule.setMode(INxtHmi.Mode.DISCONNECT);
        hmiModule.disconnect();

        while(hmiModule.isConnected()){
            //wait until disconnected
        }
        hmiModule = null;
    }
}