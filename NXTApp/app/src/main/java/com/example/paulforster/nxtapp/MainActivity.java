package com.example.paulforster.nxtapp;

import android.bluetooth.BluetoothAdapter;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.annotation.NonNull;
import android.support.design.widget.BottomNavigationView;
import android.support.v4.app.FragmentTransaction;
import android.support.v7.app.AppCompatActivity;
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
        refreshMenu();
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
                                        "Nächste Parklücke wird angefahren", Toast.LENGTH_SHORT);
                                toast.show();
                                break;
                        }
                        return false;
                    }
                });
        btDialog = new BluetoothDialogFragment();
        MapFragment mapFragment = new MapFragment();
        FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
        ft.add(R.id.content, mapFragment, "MAPFRAGMENT");
        ft.commit();
}

    @Override
    protected void onResume() {
        super.onResume();
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

    //TODO einen StatusChangeListener bauen... um Ressourcen zu sparen



    public void refreshMenu(){
    new Timer().schedule(new TimerTask() {

        @Override
        public void run() {

            runOnUiThread(new Runnable() {
                public void run() {
                    BottomNavigationView bottomNavigationView = (BottomNavigationView)
                            findViewById(R.id.bottom_navigation);
                    if (hmiModule != null)
                        if(hmiModule.isConnected()) {
                            if(hmiModule.getCurrentStatus() != null) {
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
                                        if ((((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler != null){
                                            String data = "MSG DATA";
                                            Message msg = (((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler.obtainMessage(0, data);
                                            msg.sendToTarget();
                                        }
                                        getSupportFragmentManager().popBackStack();
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
                                        if(getSupportFragmentManager().getBackStackEntryCount()==0){
                                            FragmentTransaction ft = getSupportFragmentManager()
                                                    .beginTransaction();
                                            ParkFragment parkFragment = new ParkFragment();
                                            ft.add(R.id.content, parkFragment);
                                            ft.addToBackStack(null);
                                            ft.commit();
                                        }
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
                                        if(getSupportFragmentManager().getBackStackEntryCount()==0){
                                            FragmentTransaction ft = getSupportFragmentManager()
                                                    .beginTransaction();
                                            ParkFragment parkFragment = new ParkFragment();
                                            ft.add(R.id.content, parkFragment);
                                            ft.addToBackStack(null);
                                            ft.commit();
                                        }
                                        break;
                                    case PARKED:
                                        bottomNavigationView.getMenu().getItem(0).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(1).setChecked(true);
                                        bottomNavigationView.getMenu().getItem(2).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(0).setEnabled(false);
                                        bottomNavigationView.getMenu().getItem(1).setEnabled(true);
                                        bottomNavigationView.getMenu().getItem(2).setEnabled(false);

                                        for(ImageView imageView : ((MapFragment)getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT")).imageviewArrayList){
                                            imageView.setClickable(false);
                                        }
                                        if ((((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler != null){
                                            String data = "MSG DATA";
                                            Message msg = (((MapFragment) getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT"))).mHandler.obtainMessage(0, data);
                                            msg.sendToTarget();
                                        }
                                        getSupportFragmentManager().popBackStack();
                                        break;

                                    case INACTIVE:
                                        bottomNavigationView.getMenu().getItem(0).setChecked(true);
                                        bottomNavigationView.getMenu().getItem(1).setChecked(false);
                                        bottomNavigationView.getMenu().getItem(2).setChecked(false);
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
                                        getSupportFragmentManager().popBackStack();
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
            });
        }
    }, 200, 100);
}
}