Uni Projekt

zur Zeit ist der PID regler des Line Followers nicht ausgereift. Deswegen gibt es eine if-Abfrage ca. bei Zeile 330 in ControlRST.java. Für Beispielprogramm: int version = 0 eintragen. Sonst kann auch der I-Anteil des PID deaktiviert werden: int version = 3, und Zeile 571 	outgoingPID = kp * e;// + 1 / ti * integralE; // PID-Regler  setzen.
Zeile 118 bei GuidanceAT auskommentiert. Kalibrierte Sensorwerte manuell bei PerceptionPMP eingetragen.
Bluetooth Code ist 2222
