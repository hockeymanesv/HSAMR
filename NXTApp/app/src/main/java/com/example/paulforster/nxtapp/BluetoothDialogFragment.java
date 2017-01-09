package com.example.paulforster.nxtapp;

import android.app.AlertDialog;
import android.app.Dialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.support.design.widget.BottomNavigationView;
import android.support.v4.app.DialogFragment;
import android.support.v4.app.FragmentTransaction;
import android.widget.ArrayAdapter;
import android.widget.Toast;

import java.util.Set;

import de.amr.plt.rcParkingRobot.AndroidHmiPLT;

import static com.example.paulforster.nxtapp.MainActivity.BTArrayAdapter;
import static com.example.paulforster.nxtapp.MainActivity.REQUEST_ENABLE_BT;
import static com.example.paulforster.nxtapp.MainActivity.hmiModule;


/**
 * Created by paulforster on 27.11.16.
 */
public class BluetoothDialogFragment extends DialogFragment {

    BluetoothAdapter mBluetoothAdapter = null;
    BluetoothDevice bluetoothDevice = null;

    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        // Use the Builder class for convenient dialog construction
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        builder.setTitle("Choose NXT");


        BTArrayAdapter = new ArrayAdapter<String>(getActivity(), android.R.layout.simple_list_item_1);
        // get paired devices
        final Set<BluetoothDevice> pairedDevicesSet = mBluetoothAdapter.getBondedDevices();
        // put it's one to the adapter
        if (pairedDevicesSet.size() > 0){
        builder.setAdapter(BTArrayAdapter, new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog, int which) {
                String adressName = BTArrayAdapter.getItem(which);
                String adress = adressName.substring(adressName.length()-17);
                Toast.makeText(getActivity(), "Verbinde mit " + adress, Toast.LENGTH_LONG)
                        .show();
                establishBTConn(adress);


                // The 'which' argument contains the index position
                // of the selected item
            }
        });
        for(BluetoothDevice device : pairedDevicesSet)
            BTArrayAdapter.add(device.getName()+ "\n" + device.getAddress());
        }
        else {
            builder.setMessage("No paired Devices");
        }
        //TODO ist finish der richtige Weg um die App zu beenden?
        builder.setNegativeButton("Beenden",new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog, int which) {
                // The 'which' argument contains the index position
                // of the selected item
                getActivity().finish();
            }
        });
        builder.setNeutralButton("Bluetooth Einstellungen",new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog, int which) {
                // The 'which' argument contains the index position
                // of the selected item
                Intent intentOpenBluetoothSettings = new Intent();
                intentOpenBluetoothSettings.setAction(android.provider.Settings.ACTION_BLUETOOTH_SETTINGS);
                startActivity(intentOpenBluetoothSettings);
            }
        });
        return builder.create();
    }

    @Override
    public void onStart() {
        super.onStart();
        BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (!mBluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            //getActivity, damit meine ID dann auch stimmt... in nem Fragment kommt die falsche raus
            getActivity().startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }
        //erste wenn das Ding erschaffen ist, kann ichs !cancelable machen
        setCancelable(false);
    }

    /**
     * Connect to the chosen device
     *
     */
    private void establishBTConn(String address){
        //get instance of the chosen bluetooth device
        bluetoothDevice = mBluetoothAdapter.getRemoteDevice(address);

        //get name and address of the device
        String btDeviceAddress = bluetoothDevice.getAddress();
        String btDeviceName = bluetoothDevice.getName();

        //instantiate client modul
        hmiModule = new AndroidHmiPLT(btDeviceName, btDeviceAddress);

        //connect to the specified device
        hmiModule.connect();

        //wait till connection really is established and
        //TODO hier ein WarteDialog... das wäre cool
        int i = 0;
        while (!hmiModule.isConnected()&& i<100000000/2) {
            i++;
        }
        //TODO hier könnte man den StatusChangeListener platzieren

    }
}