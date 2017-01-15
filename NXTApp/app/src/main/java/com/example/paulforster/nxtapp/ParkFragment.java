package com.example.paulforster.nxtapp;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import java.util.Timer;
import java.util.TimerTask;
import static com.example.paulforster.nxtapp.MainActivity.hmiModule;

/**
 * Verändert periodisch die Alphakanäle der Abstandsgrafik im park_fragment layout
 * @author paulforster
 */
public class ParkFragment extends Fragment {
    Timer refreshTimer;
    TimerTask refreshTimerTask;

    /**
     *  ActivityLifecycle.
     * @param inflater
     * @param container
     * @param savedInstanceState
     * @return
     */
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.park_fragment, container, false);
        return view;
    }

    /**
     *  ActivityLifecycle.
     */
    @Override
    public void onResume() {
        super.onResume();
        reScheduleTimer(100);
    }

    /**
     *  ActivityLifecycle.
     */
    @Override
    public void onPause() {
        super.onPause();
        refreshTimer.cancel();
    }

    /**
     *  ActivityLifecycle.
     */
    @Override
    public void onStop() {
        super.onStop();
        refreshTimer.purge();
    }

    /**
     * Startet den Timer für {@link #refreshDistance()} erneut
     *
     * @param duration gibt an mit welcher Periode das Menu aktualisiert werden soll
     */
    public void reScheduleTimer(int duration) {
        refreshTimer = new Timer("alertTimer",true);
        refreshTimerTask = new MyTimerTask();
        refreshTimer.schedule(refreshTimerTask, 0, duration);
    }

    /**
     * TimerTask beinhaltet führt {@link #refreshDistance()} aus
     */
    private class MyTimerTask extends TimerTask {
        @Override
        public void run() {
            getActivity().runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    refreshDistance();
                }
            });
        }
    }

    /**
     * Berechnet Alphawert in Abhängigkeit der Stufe und Entfernung
     * @param distance
     * @param step
     * @return
     */
    private float calculateAlpha (double distance, int step){
        float alpha = (float)((distance -300) * 2.1 / (50-300));
        if (alpha < step*0.7f) return 0;
        else return alpha-(step*0.7f);
    }

    /**
     * Aktualisiert Alphakanäle der Abstandsbalken
     */
    public void refreshDistance(){
        if (hmiModule != null) {
            if (hmiModule.isConnected()) {
                getView().findViewById(R.id.abstand_oben3)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceFront(), 0));
                getView().findViewById(R.id.abstand_oben2)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceFront(), 1));
                getView().findViewById(R.id.abstand_oben1)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceFront(), 2));

                getView().findViewById(R.id.abstand_seiteoben3)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceFrontSide(), 0));
                getView().findViewById(R.id.abstand_seiteoben2)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceFrontSide(), 1));
                getView().findViewById(R.id.abstand_seiteoben1)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceFrontSide(), 2));

                getView().findViewById(R.id.abstand_seiteunten3)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceBackSide(), 0));
                getView().findViewById(R.id.abstand_seiteunten2)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceBackSide(), 1));
                getView().findViewById(R.id.abstand_seiteunten1)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceBackSide(), 2));

                getView().findViewById(R.id.abstand_unten3)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceBack(), 0));
                getView().findViewById(R.id.abstand_unten2)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceBack(), 1));
                getView().findViewById(R.id.abstand_unten1)
                        .setAlpha(0.3f + calculateAlpha(hmiModule.getPosition().getDistanceBack(), 2));
            }
        }
    }
}