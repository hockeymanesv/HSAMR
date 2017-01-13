package com.example.paulforster.nxtapp;

import android.graphics.Paint;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.Parcel;
import android.os.Parcelable;
import android.support.design.widget.BottomNavigationView;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.animation.AlphaAnimation;
import android.view.animation.Animation;
import android.view.animation.LinearInterpolator;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.Toast;

import com.qozix.tileview.TileView;
import com.qozix.tileview.paths.CompositePathView;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import de.amr.plt.rcParkingRobot.AndroidHmiPLT;
import de.amr.plt.rcParkingRobot.IAndroidHmi;
import parkingRobot.INxtHmi;
import parkingRobot.hsamr0.GuidanceAT;
import parkingRobot.hsamr0.HmiPLT;

import static com.example.paulforster.nxtapp.MainActivity.hmiModule;
import static java.lang.Math.abs;

/**
 * Created by paulforster on 27.11.16.
 */

public class MapFragment extends Fragment {

    Timer refreshTimer;
    TimerTask refreshTimerTask;
    static TileView tileView = null;
    private ImageView robot = null;
    private Paint linePaint = null;
    private CircularArrayList roboPath = null;
    private CompositePathView.DrawablePath drawnPath = null;
    //Parkingslot-Kram
    private int noStoredSlots = 0;
    private int noSlots = 0;
    ArrayList<IAndroidHmi.ParkingSlot> parkingslotArrayList = new ArrayList<>();
    ArrayList<ImageView> imageviewArrayList = new ArrayList<>();

    Handler mHandler = null;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.map_fragment, container, false);
        /**
        if (savedInstanceState != null) {
            // Restore last state
            parkingslotArrayList = savedInstanceState.getParcelableArrayList("parkingslotArrayList");
            //TODO die Dinger müssen nochmal gezeichnet werden
        } else {
            parkingslotArrayList = new ArrayList<>();
        }
         */
        tileView = new TileView(this.getActivity());
        tileView.setSize(3072, 1654);
        tileView.defineBounds(-40,100,220,-40);
        tileView.addDetailLevel(1f,"robomap/tile-%d_%d.png");
        tileView.setScale(0.34f);
        robot = new ImageView(this.getActivity());
        robot.setImageResource(R.drawable.robo);
        tileView.addMarker(robot, 0, 0, -0.5f, -0.5f);
        tileView.moveToMarker(robot, true);

        robot.setRotation(-90);
        //muss wahrscheinlich so, da die ID erst zur Laufzeit festgelegt wird
        ((RelativeLayout)view.findViewById(R.id.map)).addView(tileView);
        roboPath = new CircularArrayList(250);
        linePaint = tileView.getDefaultPathPaint();
        linePaint.setColor(getResources().getColor(R.color.colorPrimaryDark));
        roboPath.add(0, new double[] {0,0});
        roboPath.remove(0);
        refreshTimer = new Timer();
        refreshTimerTask = new TimerTask() {
            @Override
            public void run() {
                getActivity().runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        refreshMap();
                    }
                });
            }
        };
        refreshMap();
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

    @Override
    public void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);
        outState.putParcelableArrayList("parkingslotArrayList", parkingslotArrayList);
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
                    refreshMap();
                }
            });
        }
    }

    private void addMarker(IAndroidHmi.ParkingSlot slot, ImageView view){
        float xa = slot.getFrontBoundaryPosition().x;
        float xe = slot.getBackBoundaryPosition().x;
        float ya = slot.getFrontBoundaryPosition().y;
        float ye = slot.getBackBoundaryPosition().y;
        Log.e("Luecke", " ID:" + Integer.toString(slot.getID()));
        if(abs(xa-xe)<3){
            tileView.addMarker(view,(xa+xe/2),(ye-ya/2),+1f,-0.5f);
            Log.e("Luecke", " Linie 2 ### Anfang:" + Float.toString(xa) + " , " + Float.toString(ya) + " Ende:" + Float.toString(xe) + " , " + Float.toString(ye));
        }else if (xa-xe>=3){
            tileView.addMarker(view,(xa+xe/2),(ye-ya/2),-0.5f,-1.5f);
            Log.e("Luecke", " Linie 3 ### Anfang:" + Float.toString(xa) + " , " + Float.toString(ya) + " Ende:" + Float.toString(xe) + " , " + Float.toString(ye));

        }else if(xa-xe<=-3){
            tileView.addMarker(view,(xa+xe/2),(ye-ya/2),-0.5f,+0.5f);
            Log.e("Luecke", " Linie 1 ### Anfang:" + Float.toString(xa) + " , " + Float.toString(ya) + " Ende:" + Float.toString(xe) + " , " + Float.toString(ye));

        }else{
            tileView.addMarker(view,(xa+xe/2),(ye-ya/2),-0.5f,-0.5f);
            Log.e("Luecke", " XXXX ### Anfang:" + Float.toString(xa) + " Ende:" + Float.toString(xe));

        }

    }

    public void refreshMap(){
                        if (hmiModule != null) if(hmiModule.isConnected()) {
                            //###########RobotPosition##############################################
                            tileView.moveMarker(robot, hmiModule.getPosition().getX(),
                                    hmiModule.getPosition().getY());
                            robot.setRotation(-hmiModule.getPosition().getAngle() - 90);
                            Log.e("Winkel", Float.toString(hmiModule.getPosition().getAngle()));

                            //###########RobotPath##################################################
                            //new RefreshPathTask().execute();
                            //TODO der Pfad muss vor der Drehung auch zwischengespeichert werden
                            tileView.removePath(drawnPath);
                            if(roboPath.size()==roboPath.capacity()){
                                roboPath.remove(0);
                            }
                            roboPath.add( new double[] { hmiModule.getPosition().getX(), hmiModule.getPosition().getY() } );
                            drawnPath = tileView.drawPath(roboPath, linePaint);

                            //###########Parkluecken################################################
                            noSlots = hmiModule.getNoOfParkingSlots();
                            //TODO noStoredSlots speichern?
                            if (noSlots > noStoredSlots) {
                                Log.e("Luecke", "Arraygröße ist " + Integer.toString(noSlots));
                                for (int i = noStoredSlots; i < noSlots; i++) {
                                    final int slotID = hmiModule.getParkingSlot(i).getID();
                                    //Wenn Parklücke unbekannt
                                    if (slotID > parkingslotArrayList.size()-1){
                                        parkingslotArrayList.add(slotID, hmiModule.getParkingSlot(i));
                                        //TODO HIER
                                        //Wenn Parklücken unbekannt
                                        final ImageView slotMarker = new ImageView(getActivity());
                                        slotMarker.setImageResource(R.drawable.parkingsign1);
                                        //Wenn parkbar
                                        if (hmiModule.getParkingSlot(i).getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.GOOD){
                                            slotMarker.setOnClickListener(
                                                    new View.OnClickListener() {
                                                        @Override
                                                        public void onClick(View v) {
                                                            hmiModule.setSelectedParkingSlot(slotID);
                                                            hmiModule.setMode(INxtHmi.Mode.PARK_THIS);
                                                            Toast toast = Toast.makeText(getActivity(),
                                                                    "Parklücke " + (slotID + 1) +
                                                                            " wird angefahren", Toast.LENGTH_SHORT);
                                                            toast.show();
                                                            final Animation animation = new AlphaAnimation(1f, 0.3f);
                                                            animation.setDuration(250);
                                                            animation.setInterpolator(new LinearInterpolator());
                                                            animation.setRepeatCount(Animation.INFINITE);
                                                            animation.setRepeatMode(Animation.REVERSE);
                                                            final Handler h = new Handler();
                                                            Runnable r = new Runnable() {
                                                                @Override
                                                                public void run() {
                                                                    slotMarker.startAnimation(animation);
                                                                    for(ImageView imageView : imageviewArrayList){
                                                                        if(imageView != imageviewArrayList.get(slotID)) imageView.clearAnimation();
                                                                    }
                                                                }
                                                            };
                                                            h.postDelayed(r, 1000);
                                                            mHandler = new Handler(){
                                                                @Override
                                                                public void handleMessage(Message msg) {
                                                                    int msgId = msg.what;
                                                                    slotMarker.clearAnimation();
                                                                }
                                                            };
                                                        }
                                                    }
                                            );
                                            slotMarker.setAlpha(1.0f);
                                        }
                                        else{
                                            slotMarker.setAlpha(0.5f);
                                        }
                                        /**
                                        tileView.addMarker(slotMarker,
                                                ((hmiModule.getParkingSlot(i).getFrontBoundaryPosition().x
                                                        + hmiModule.getParkingSlot(i).getBackBoundaryPosition().x) / 2),
                                                ((hmiModule.getParkingSlot(i).getBackBoundaryPosition().y
                                                        - hmiModule.getParkingSlot(i).getFrontBoundaryPosition().y) / 2),
                                                -0.5f, -0.5f);
                                         */
                                        addMarker(hmiModule.getParkingSlot(i), slotMarker);
                                        Log.e("testtest", " slotID: " + Integer.toString(slotID) + " i: " + Integer.toString(i) + " noSlots: " + Integer.toString(noSlots) + " nostored: " + Integer.toString(noStoredSlots));
                                        imageviewArrayList.add(slotID, slotMarker);
                                    }
                                    //Wenn Parklücke bereits bekannt
                                    else {
                                        final ImageView slotMarker = imageviewArrayList.get(slotID);
                                        if(parkingslotArrayList.get(slotID).getParkingSlotStatus() != hmiModule.getParkingSlot(i).getParkingSlotStatus()){
                                            if (hmiModule.getParkingSlot(i).getParkingSlotStatus() == IAndroidHmi.ParkingSlot.ParkingSlotStatus.GOOD){
                                                slotMarker.setOnClickListener(
                                                        new View.OnClickListener() {
                                                            @Override
                                                            public void onClick(View v) {
                                                                hmiModule.setSelectedParkingSlot(slotID);
                                                                hmiModule.setMode(INxtHmi.Mode.PARK_THIS);
                                                                Toast toast = Toast.makeText(getActivity(),
                                                                        "Parklücke " + (slotID + 1) +
                                                                                " wird angefahren", Toast.LENGTH_SHORT);
                                                                toast.show();
                                                                final Animation animation = new AlphaAnimation(1f, 0.3f);
                                                                animation.setDuration(250);
                                                                animation.setInterpolator(new LinearInterpolator());
                                                                animation.setRepeatCount(Animation.INFINITE);
                                                                animation.setRepeatMode(Animation.REVERSE);
                                                                final Handler h = new Handler();
                                                                Runnable r = new Runnable() {
                                                                    @Override
                                                                    public void run() {
                                                                        slotMarker.startAnimation(animation);
                                                                        for(ImageView imageView : imageviewArrayList){
                                                                            if(imageView != imageviewArrayList.get(slotID))  imageView.clearAnimation();
                                                                        }
                                                                    }
                                                                };
                                                                h.postDelayed(r, 1000);
                                                                mHandler = new Handler(){
                                                                    @Override
                                                                    public void handleMessage(Message msg) {
                                                                        int msgId = msg.what;
                                                                        slotMarker.clearAnimation();
                                                                    }
                                                                };
                                                            }
                                                        }
                                                );
                                                slotMarker.setAlpha(1.0f);
                                            }
                                            else{
                                                slotMarker.setAlpha(0.5f);
                                            }
                                        }
                                        parkingslotArrayList.add(slotID, hmiModule.getParkingSlot(i));
                                        Log.e("Luecke", "ID " + Integer.toString(slotID) + " aktualisiert");

                                        tileView.moveMarker(slotMarker,
                                                ((hmiModule.getParkingSlot(i).getFrontBoundaryPosition().x
                                                        + hmiModule.getParkingSlot(i).getBackBoundaryPosition().x) / 2),
                                                ((hmiModule.getParkingSlot(i).getBackBoundaryPosition().y
                                                        - hmiModule.getParkingSlot(i).getFrontBoundaryPosition().y) / 2));

                                    }

/**
                                    parkingslotArrayList.add(hmiModule.getParkingSlot(i).getID(), hmiModule.getParkingSlot(i));
                                    final ImageView slotMarker = new ImageView(getActivity());
                                    slotMarker.setImageResource(R.drawable.parkingsign1);
                                    final int slotNumber = hmiModule.getParkingSlot(i).getID();
                                    if (hmiModule.getParkingSlot(i).getParkingSlotStatus() ==
                                            IAndroidHmi.ParkingSlot.ParkingSlotStatus.GOOD) {
                                        slotMarker.setOnClickListener(
                                                new View.OnClickListener() {
                                                    @Override
                                                    public void onClick(View v) {
                                                        hmiModule.setSelectedParkingSlot(slotNumber);
                                                        hmiModule.setMode(INxtHmi.Mode.PARK_THIS);
                                                        Toast toast = Toast.makeText(getActivity(),
                                                                "Parklücke " + (slotNumber + 1) +
                                                                        " wird angefahren", Toast.LENGTH_SHORT);
                                                        toast.show();
                                                        final Animation animation = new AlphaAnimation(1f, 0.3f);
                                                        animation.setDuration(250);
                                                        animation.setInterpolator(new LinearInterpolator());
                                                        animation.setRepeatCount(Animation.INFINITE);
                                                        animation.setRepeatMode(Animation.REVERSE);
                                                        final Handler h = new Handler();
                                                        Runnable r = new Runnable() {
                                                            @Override
                                                            public void run() {
                                                                slotMarker.startAnimation(animation);
                                                                for(ImageView imageView : imageviewArrayList){
                                                                    if(imageView != imageviewArrayList.get(slotNumber)) imageView.clearAnimation();
                                                                }
                                                            }
                                                        };
                                                        h.postDelayed(r, 1000);
                                                        mHandler = new Handler(){
                                                            @Override
                                                            public void handleMessage(Message msg) {
                                                                int msgId = msg.what;
                                                                slotMarker.clearAnimation();
                                                            }
                                                        };
                                                    }
                                                }
                                        );
                                        slotMarker.setAlpha(1.0f);
                                    } else {
                                        slotMarker.setAlpha(0.5f);
                                    }
                                    tileView.addMarker(slotMarker,
                                            ((hmiModule.getParkingSlot(i).getFrontBoundaryPosition().x
                                                    + hmiModule.getParkingSlot(i).getBackBoundaryPosition().x) / 2),
                                            ((hmiModule.getParkingSlot(i).getBackBoundaryPosition().y
                                                    - hmiModule.getParkingSlot(i).getFrontBoundaryPosition().y) / 2),
                                            -0.5f, -0.5f);
                                    imageviewArrayList.add(slotMarker);*/
                                }
                                noStoredSlots = noSlots;
                                //tileView die Marker zeichnen lassen
                                for (IAndroidHmi.ParkingSlot parkingSlot : parkingslotArrayList) {
                                    tileView.moveMarker(imageviewArrayList.get(parkingSlot.getID()),
                                            ((parkingSlot.getBackBoundaryPosition().x +
                                                    parkingSlot.getFrontBoundaryPosition().x) / 2),
                                            ((parkingSlot.getBackBoundaryPosition().y +
                                                    parkingSlot.getFrontBoundaryPosition().y) / 2));
                                }
                            }

                        }
                    }


}