package com.example.paulforster.nxtapp;

import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import java.util.Timer;
import java.util.TimerTask;

import static com.example.paulforster.nxtapp.MainActivity.hmiModule;

/**
 * Created by paulforster on 04.12.16.
 */

public class ParkFragment extends Fragment {
    Timer refreshTimer;
    TimerTask refreshTimerTask;
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.park_fragment, container, false);
        return view;
    }

    @Override
    public void onResume() {
        super.onResume();
        reScheduleTimer(100);
    }

    @Override
    public void onPause() {
        super.onPause();
        refreshTimer.cancel();
    }

    @Override
    public void onStop() {
        super.onStop();
        refreshTimer.purge();
    }

    public void reScheduleTimer(int duration) {
        refreshTimer = new Timer("alertTimer",true);
        refreshTimerTask = new MyTimerTask();
        refreshTimer.schedule(refreshTimerTask, 0, duration);
    }

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

    public void refreshDistance(){
                            if (hmiModule != null) {
                                if (hmiModule.isConnected()) {
                                    double visualDistance;
                                    visualDistance = (hmiModule.getPosition().getDistanceFront()
                                            - 300) * (3 - 0) / (50 - 300) + 0;

                                    getView().findViewById(R.id.abstand_oben3)
                                            .setAlpha((float) visualDistance);
                                    getView().findViewById(R.id.abstand_oben2)
                                            .setAlpha((float) visualDistance - 1);
                                    getView().findViewById(R.id.abstand_oben1)
                                            .setAlpha((float) visualDistance - 2);

                                    visualDistance = (hmiModule.getPosition().getDistanceFrontSide()
                                            - 300) * (3 - 0) / (50 - 300) + 0;
                                    getView().findViewById(R.id.abstand_seiteoben3)
                                            .setAlpha((float) visualDistance);
                                    getView().findViewById(R.id.abstand_seiteoben2)
                                            .setAlpha((float) visualDistance - 1);
                                    getView().findViewById(R.id.abstand_seiteoben1)
                                            .setAlpha((float) visualDistance - 2);

                                    visualDistance = (hmiModule.getPosition().getDistanceBackSide()
                                            - 300) * (3 - 0) / (50 - 300) + 0;
                                    getView().findViewById(R.id.abstand_seiteunten3)
                                            .setAlpha((float) visualDistance);
                                    getView().findViewById(R.id.abstand_seiteunten2)
                                            .setAlpha((float) visualDistance - 1);
                                    getView().findViewById(R.id.abstand_seiteunten1)
                                            .setAlpha((float) visualDistance - 2);

                                    visualDistance = (hmiModule.getPosition().getDistanceBack()
                                            - 300) * (3 - 0) / (50 - 300) + 0;
                                    getView().findViewById(R.id.abstand_unten3)
                                            .setAlpha((float) visualDistance);
                                    getView().findViewById(R.id.abstand_unten2)
                                            .setAlpha((float) visualDistance - 1);
                                    getView().findViewById(R.id.abstand_unten1)
                                            .setAlpha((float) visualDistance - 2);
                                }
                            }
                        }
}