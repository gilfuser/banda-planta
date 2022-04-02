
(
    OSCdef(\p0, { |msg|
	// Ndef(\p0).value(msg);
    MFdef(\p0).value(msg);
    // "chegou na porta 0! % \n".postf(msg);
}, '/suculenta', recvPort: 57120 ).permanent_(true);
    OSCdef(\p1, { |msg|
	// Ndef(\p0).value(msg);
	MFdef(\p1).value(msg);
    // "chegou na porta 1! % \n".postf(msg);
}, '/suculenta', recvPort: 57121 ).permanent_(true);
    OSCdef(\p2, { |msg|
	// Ndef(\p0).value(msg);
    MFdef(\p2).value(msg);
    // "chegou na porta 2! % \n".postf(msg);
}, '/suculenta', recvPort: 57122 ).permanent_(true)
);

// r = OSCFunc({ arg msg, time; [time, msg].postln }, '/good/news', n);

thisProcess.openUDPPort(57121);

thisProcess.openPorts; // list all open ports

q.bromelia = q.bromelia ? ();
