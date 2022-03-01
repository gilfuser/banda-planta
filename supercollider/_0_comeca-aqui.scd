Ndef(\theHub, {|msg| msg  } )
Ndef(\theBrom, {|msg| msg }).kr//set(\msg, 0.0125)
Ndef(\theBrom).addSpec(\msg, [0, 667])
Ndef(\theSuc, {|msg| msg }).kr//set(\msg, 0.0125)
Ndef(\theSuc).addSpec(\msg, [0, 667])

(
OSCdef(\oscFromSuculenta, { |msg|
	Ndef(\theSuc).value(msg);
	MFdef(\theSuc).value(msg);
}, '/suculenta', recvPort: 57121 ).permanent_(true)
);

(
OSCdef(\oscFromBromelia, { |msg|
	Ndef(\theBrom).value(msg);
	MFdef(\theBrom).value(msg);
}, '/bromelia' ).permanent_(true)
)

x = Bus.control(s).set(1)
o = OSCMon.new
o.show
x.set(2)
x.get

MFdef(\theSuc).add(\post, {|msg| msg.postln});
MFdef(\theSuc).enable(\post);
MFdef(\theSuc).disable(\post);

MFdef(\theBrom).add(\post, {|msg| msg.postln});
MFdef(\theBrom).enable(\post);
MFdef(\theBrom).disable(\post);

MFdef(\theHub).add(\post, {|msg| msg.postln});
MFdef(\theHub).enable(\post);
MFdef(\theHub).disable(\post);

MFdef(\theBrom).add(\bromelia, {|msg| Ndef(\theBrom).set(\msg, msg[1]) });
MFdef(\theHub).enable(\bromelia);
MFdef(\theHub).disable(\bromelia);

thisProcess.openUDPPort(57121);
thisProcess.openUDPPort(57122);
thisProcess.openUDPPort(57123);
thisProcess.openPorts; // list all open ports





