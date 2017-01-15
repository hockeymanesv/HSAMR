package com.example.paulforster.nxtapp;

import android.bluetooth.BluetoothAdapter;
import android.content.Intent;
import android.os.Bundle;
import android.os.Message;
import android.os.PersistableBundle;
import android.support.annotation.NonNull;
import android.support.design.widget.BottomNavigationView;
import android.support.v4.app.FragmentTransaction;
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
import parkingRobot.hsamr0.GuidanceAT;

import static com.example.paulforster.nxtapp.MapFragment.tileView;

/**
 * Where all the magic happens!
 * @author paulforster
 */
public class MainActivity extends AppCompatActivity {

    static final int REQUEST_ENABLE_BT = 154;
    static AndroidHmiPLT hmiModule = null;
    static ArrayAdapter<String> BTArrayAdapter;
    static BluetoothDialogFragment btDialog = null;
    static Timer refreshTimer;
    static TimerTask refreshTimerTask;

    /**
     * ActivityLifecycle.
     * @param savedInstanceState
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main );
        //Kein Bluetooth --> keine App
        BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth nicht verfügbar!",
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
        btDialog = new BluetoothDialogFragment();
        MapFragment mapFragment = new MapFragment();
        ParkFragment parkFragment = new ParkFragment();
        FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
        ft.add(R.id.content, mapFragment, "MAPFRAGMENT");
        ft.add(R.id.content, parkFragment, "PARKFRAGMENT");
        ft.hide(parkFragment);
        ft.commit();
}

    /**
     * ActivityLifecycle.
     * @param outState
     * @param outPersistentState
     */
    @Override
    public void onSaveInstanceState(Bundle outState, PersistableBundle outPersistentState) {
        super.onSaveInstanceState(outState, outPersistentState);
    }

    /**
     * ActivityLifecycle.
     */
    @Override
    protected void onStart() {
        super.onStart();

        Log.e("AConPause", "onStart");

        reScheduleTimer(100);
    }

    /**
     * ActivityLifecycle.
     */
    @Override
    protected void onPause() {
        super.onStop();
        Log.e("AConPause", "refreshTimer.cancel() happens");
        refreshTimer.cancel();
        refreshTimer = null;
    }

    /**
     * ActivityLifecycle.
     */
    @Override
    protected void onResume() {
        super.onResume();

        Log.e("AConPause", "onResume");
        //TODO wenn man gerade Bluetooth eingeschaltet hat, findet er die Liste noch nicht...
        if (!btDialog.isAdded()) if(hmiModule == null) btDialog.show(getSupportFragmentManager(), "btDialog");
        else if (!hmiModule.isConnected()) btDialog.show(getSupportFragmentManager(), "btDialog");
    }

    /**
     * Beendet die App für den Fall, dass der Nutzer wider Erwarten BT deaktiviert hat.
     * @param requestCode
     * @param resultCode
     * @param data
     */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if(requestCode == REQUEST_ENABLE_BT){
            if(resultCode != RESULT_OK){
                finish();
            }
        }
    }

    /**
     * Startet den Timer für {@link #refreshMenu()} erneut
     *
     * @param duration gibt an mit welcher Periode das Menu aktualisiert werden soll
     */
    public void reScheduleTimer(int duration) {
        refreshTimer = new Timer();
        refreshTimerTask = new MyTimerTask();
        refreshTimer.schedule(refreshTimerTask, 0, duration);
    }

    /**
     * TimerTask beinhaltet führt {@link #refreshMenu()} aus
     */
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

    /**
     * Aktualisiert die Ansicht der BottomNavigation abhängig vom {@link #hmiModule}
     * Ruft die bestehende Instanz von {@link ParkFragment} auf, wenn geparkt wird.
     */
    public void refreshMenu(){
        BottomNavigationView bottomNavigationView = (BottomNavigationView)
                findViewById(R.id.bottom_navigation);
        FragmentTransaction fragmentTransaction = getSupportFragmentManager().beginTransaction();
        if (hmiModule != null)
            if(hmiModule.isConnected()) {
                Log.e("Status", "is verbunden");
                if(hmiModule.getCurrentStatus() != null) {
                    Log.e("Status", "is da" + Integer.toString(hmiModule.getCurrentStatus().ordinal()));
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
                            tileView.slideToAndCenter(hmiModule.getPosition().getX(), hmiModule.getPosition().getY());
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
                            break;
                        case PARK_THIS:
                            bottomNavigationView.getMenu().getItem(0).setChecked(false);
                            bottomNavigationView.getMenu().getItem(1).setChecked(false);
                            bottomNavigationView.getMenu().getItem(2).setChecked(true);
                            bottomNavigationView.getMenu().getItem(0).setEnabled(true);
                            bottomNavigationView.getMenu().getItem(1).setEnabled(true);
                            bottomNavigationView.getMenu().getItem(2).setEnabled(true);
                            tileView.slideToAndCenter(hmiModule.getPosition().getX(), hmiModule.getPosition().getY());
                            for(ImageView imageView : ((MapFragment)getSupportFragmentManager().findFragmentByTag("MAPFRAGMENT")).imageviewArrayList){
                                imageView.setClickable(true);
                            }
                            fragmentTransaction.show((ParkFragment)getSupportFragmentManager().findFragmentByTag("PARKFRAGMENT"));
                            fragmentTransaction.commit();
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
                        case EXIT:
                            //terminateBluetoothConnection();
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

    /**
     * Ein Klick auf return führt  {@link #terminateBluetoothConnection()} aus
     */
    @Override
    public void onBackPressed() {
        super.onBackPressed();
        if (hmiModule != null && hmiModule.connected) {

            terminateBluetoothConnection();
        }
    }

    /**
     * Beendet die BluetoothVerbindung. Zuvor wird jedoch der Robotermodus auf DISCONNECT gesetzt.
     *
     * Leider hält die vorgegebene Klasse nicht viel vom Multithreading und lässt einen NXTCommAndroid$ReadThread zurück.
     * Seltsamerweise lässt dieser sich auch nicht aus der App heraus über die PID killn.
     */
    private void terminateBluetoothConnection(){
        Toast.makeText(this, "Bluetoothverbindung beendet!", Toast.LENGTH_LONG).show();
        hmiModule.setMode(INxtHmi.Mode.DISCONNECT);
        while(hmiModule.getCurrentStatus() != GuidanceAT.CurrentStatus.EXIT){}
        hmiModule.disconnect();
        while(hmiModule.isConnected()){
            //wait until disconnected
        }
        hmiModule = null;
        finish();
    }
}