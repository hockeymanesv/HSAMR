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
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.park_fragment, container, false);
        refreshDistance();
        return view;
    }

    public void refreshDistance(){
        new Timer().schedule(new TimerTask() {

            @Override
            public void run() {
                if (isVisible()) {
                    getActivity().runOnUiThread(new Runnable() {
                        public void run() {
                            if (hmiModule != null) {
                                if (hmiModule.isConnected()) {
                                    double visualDistance;
                                    visualDistance = (hmiModule.getPosition().getDistanceFront()
                                            - 300) * (3 - 0) / (50 - 300) + 0;
                                    getActivity().findViewById(R.id.abstand_oben3)
                                            .setAlpha((float) visualDistance);
                                    getActivity().findViewById(R.id.abstand_oben2)
                                            .setAlpha((float) visualDistance - 1);
                                    getActivity().findViewById(R.id.abstand_oben1)
                                            .setAlpha((float) visualDistance - 2);

                                    visualDistance = (hmiModule.getPosition().getDistanceFrontSide()
                                            - 300) * (3 - 0) / (50 - 300) + 0;
                                    getActivity().findViewById(R.id.abstand_seiteoben3)
                                            .setAlpha((float) visualDistance);
                                    getActivity().findViewById(R.id.abstand_seiteoben2)
                                            .setAlpha((float) visualDistance - 1);
                                    getActivity().findViewById(R.id.abstand_seiteoben1)
                                            .setAlpha((float) visualDistance - 2);

                                    visualDistance = (hmiModule.getPosition().getDistanceBackSide()
                                            - 300) * (3 - 0) / (50 - 300) + 0;
                                    getActivity().findViewById(R.id.abstand_seiteunten3)
                                            .setAlpha((float) visualDistance);
                                    getActivity().findViewById(R.id.abstand_seiteunten2)
                                            .setAlpha((float) visualDistance - 1);
                                    getActivity().findViewById(R.id.abstand_seiteunten1)
                                            .setAlpha((float) visualDistance - 2);

                                    visualDistance = (hmiModule.getPosition().getDistanceBack()
                                            - 300) * (3 - 0) / (50 - 300) + 0;
                                    getActivity().findViewById(R.id.abstand_unten3)
                                            .setAlpha((float) visualDistance);
                                    getActivity().findViewById(R.id.abstand_unten2)
                                            .setAlpha((float) visualDistance - 1);
                                    getActivity().findViewById(R.id.abstand_unten1)
                                            .setAlpha((float) visualDistance - 2);
                                }
                            }
                        }
                    });
                }
            }
        }, 200, 100);
    }
}