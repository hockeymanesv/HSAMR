package com.example.paulforster.nxtapp;

import android.graphics.Paint;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
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

/**
 * Created by paulforster on 27.11.16.
 */



//TODO Warum wäre hier kein static erlaubt?
public class MapFragment extends Fragment {


    //TODO kann man die View auch über ihre ID's teilen?
    private TileView tileView = null;
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
        tileView = new TileView(this.getActivity());
        tileView.setSize(1024, 512);
        tileView.defineBounds(-30,90,210,-30);
        tileView.addDetailLevel(1f,"robomap/tile-%d_%d.png");
        robot = new ImageView(this.getActivity());
        robot.setImageResource(R.drawable.robo);
        tileView.addMarker(robot, 0, 0, -0.5f, -0.5f);
        robot.setRotation(-90);
        //muss wahrscheinlich so, da die ID erst zur Laufzeit festgelegt wird
        ((RelativeLayout)view.findViewById(R.id.map_main)).addView(tileView);
        //TODO erstmal die ID festgehalten
        roboPath = new CircularArrayList(250);
        linePaint = tileView.getDefaultPathPaint();
        linePaint.setColor(getResources().getColor(R.color.colorPrimaryDark));
        roboPath.add(0, new double[] {0,0});
        roboPath.remove(0);
        refreshMap();
        return view;
    }


    final Handler updateBot = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            if(msg.what==0){
                int i;
            }
            super.handleMessage(msg);
        }
    };

    final Handler updatePath = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            if(msg.what==0){
                int i;
            }
            super.handleMessage(msg);
        }
    };

    final Handler setParkIcon = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            if(msg.what==0){
                int i;
            }
            super.handleMessage(msg);
        }
    };

    final Handler updateParkIcon = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            if(msg.what==0){
                int i;
            }
            super.handleMessage(msg);
        }
    };


    private class RefreshPathTask extends AsyncTask<Void, Void, Void> {
        protected Void doInBackground(Void... params) {
            if(roboPath.size()==roboPath.capacity()){
                roboPath.remove(0);
            }
            roboPath.add( new double[] { hmiModule.getPosition().getX(), hmiModule.getPosition().getY() } );
            return null;
        }
        protected void onPostExecute(Void result) {
            drawnPath = tileView.drawPath(roboPath, linePaint);
        }
        protected void onPreExecute() {
            tileView.removePath(drawnPath);
        }
    }


    public void refreshMap(){
        new Timer().schedule(new TimerTask() {

            @Override
            public void run() {

                getActivity().runOnUiThread(new Runnable() {
                    public void run() {
                        if (hmiModule != null) if(hmiModule.isConnected()) {
                            //###########RobotPosition##############################################
                            tileView.moveMarker(robot, hmiModule.getPosition().getX(),
                                    hmiModule.getPosition().getY());
                            robot.setRotation(-hmiModule.getPosition().getAngle() - 90);

                            //###########RobotPath##################################################
                            //new RefreshPathTask().execute();

                            tileView.removePath(drawnPath);
                            if(roboPath.size()==roboPath.capacity()){
                                roboPath.remove(0);
                            }
                            roboPath.add( new double[] { hmiModule.getPosition().getX(), hmiModule.getPosition().getY() } );
                            drawnPath = tileView.drawPath(roboPath, linePaint);

                            //###########Parkluecken################################################
                            noSlots = hmiModule.getNoOfParkingSlots();
                            if (noSlots > noStoredSlots) {
                                for (int i = noStoredSlots; i < noSlots; i++) {
                                    parkingslotArrayList.add(hmiModule.getParkingSlot(i));
                                    final ImageView slotMarker = new ImageView(getActivity());
                                    slotMarker.setImageResource(R.drawable.parkingsign1);
                                    final int slotNumber = i;
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
                                                        //slotMarker.startAnimation(animation);

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
                                                                //Log.d("HANDLER","GOT msg : " + msgId);
                                                                slotMarker.clearAnimation();
                                                            }
                                                        };

                                                        //TODO hier muss ein Handler rein, der die Animation wieder beendet.
/**
                                                        hmiModule.setStatusChangeListener(new StatusChangeListener() {
                                                            @Override public void onChange() {
                                                                slotMarker.clearAnimation();
                                                            }
                                                        });


                                                        final Handler h = new Handler();
                                                        Runnable r = new Runnable() {
                                                            @Override
                                                            public void run() {
                                                                for(ImageView imageView : imageviewArrayList){
                                                                    if(imageView != imageviewArrayList.get(slotNumber)) imageView.clearAnimation();
                                                                }
                                                            }
                                                        };
                                                        h.postDelayed(r, 1000);

*/


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
                                    imageviewArrayList.add(slotMarker);
                                }
                                noStoredSlots = noSlots;
                            }
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
                });
            }
        }, 200, 100);
    }
    //TODO Linie verschwindet erst, wenn das Array voll ist
}