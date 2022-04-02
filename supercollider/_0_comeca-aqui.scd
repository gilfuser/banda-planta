HelpBrowser.instance
StartupFile.choose
a = Pbind(amp: 0.95).play
r = Recorder(s)
s.boot
s.quit
s.meter

p.push
~test.fadeTime = 16
~test = { SinOscFB.ar([222,555], SinOsc.kr(0.125).range(0,1)) * 0.5 }
~test.stop(16)


/////////////////////////////////////////////////////////////////////////////////////////

MFdef(\p0).add(\post, { |msg| msg[1].postln });
MFdef(\p0).disable(\post);
MFdef(\p0).enable(\post);

Ndef(\say1).play

CmdPeriod.run


p.plantax.play

p

p.planta1 = { SinOscFB.ar( ( q.vMoveOffset.kr.abs ).linexp(0, 4, 440, 110),  q.vNormal.kr/*, q.vNormal.kr.lag(0.2) * 0.5*/,  mul: \amp.kr(0.125) ) }
p.planta1.stop(4)

(
Ndef(\planta, { | pan = 1, cut = 4000, rez = 0.75, amp = 2 dist = 0 lagt = 0.5 wform = 0 att = 0.02 dec = 0.5, sustain = 1 |
    var freq = q.vNormal.kr.linexp(0,1,88, LFNoise2.kr(0.125).exprange(440,880)  );
	var waves = [ DPW4Saw.ar( freq.lag2(0.5), 0.75 ),  Pulse.ar( freq.lag2(0.5) ) ];
	var sound = SelectX.ar( q.vNormal.kr(1), waves ); //!( s.options.numOutputBusChannels );
    var gate = Changed.kr( q.vMoveOffset.kr );
	var env = EnvGen.kr( Env.adsr( att, dec, \sus.kr(0.5), \rel.kr(0.1) ), gate, timeScale: sustain, doneAction: 0 );
	var noise = { LFDNoise3.ar(0.3,0.0156,1) };
	sound = sound * noise;
	sound = ( RLPFD.ar( sound, cut, rez, dist ) * 0.5 ) + ( RLPF.ar(sound, cut) * 0.5 );
	sound = sound + ( HPF.ar(sound, 400, 10).softclip * 0.04);
	sound = sound + PitchShift.ar(sound, 0.1, 1.0, 0.02, 0.2) * amp * env;
	sound = sound.softclip * 6;
    Limiter.ar(sound)
    // sig = LeakDC.ar(sig)
}).play
)

(
Ndef(\dubba, { |att = 0.02 rel = 0.2 freq = 220 t_trig = 1 |
	var sig, trig, frecip, env;
	trig = Trig1.kr( /*q.vNormal.kr.lag(1) */ Changed.kr( q.vNormal.kr ) );
	// var freq = ZeroCrossing.kr( \freqr.kr.lag(0.2) ).lag(1);
    frecip = ( q.vMoveOffset.kr.abs.lag(1) + 0.001 ).reciprocal; //.clip(0.001, 1);
	// var env = Env.perc( 0.001, frecip * LFNoise2.kr(0.12).range(1, 1.25) ).ar(gate: trig);
	env = EnvGen.kr(Env.adsr( att, 0.2, 0.5, rel, curve: [4,-4] ), Trig1.kr(trig, att + rel ), timeScale: 4.reciprocal, doneAction: 0 );
    sig = SinOsc.ar( ( Lag3UD.kr( trig, frecip / 1000, frecip, q.vNormal.kr.lag2(0.4) + 0.1 ) ).exprange( 0.22, 220 ) * [-0.1, 0, 0.1].midiratio, 0.5pi );
	sig = sig * env;
	// sig = ( sig * Lag3UD.kr( trig, 0.001, LFNoise2.kr(0.12).range(1.2, 1.7) ).exprange( 1, LFNoise2.kr(0.12).range(1.7, 2.4) ) ).tanh;
	sig = BPeakEQ.ar( sig, LFCub.kr(0.12, 1.5pi).exprange( 61.74, 164.81 ), 0.3, 2 );
	sig = (sig * 4).tanh;
	// sig = BPeakEQ.ar(sig, LFCub.kr(0.08, 0.5pi).exprange( 73.4, 2794 ), 0.4, 6);
	sig = (sig * Lag3UD.kr( trig, 0.001, LFNoise2.kr(0.06).range( 0.5, 3) ).exprange(1, LFNoise2.kr(0.1).range(3, 10) ) ).tanh;
	sig = Compander.ar( sig, sig, 0.1, 1, 1/5, 0.05, 0.2 );
	// sig = HPF2Casc.ar( sig, 21.827, 10 );
	sig = BLowPass4.ar( sig, Lag3UD.kr( trig, 0.01, 1 ).exprange(61.74, 11175.303), LFNoise2.kr(0.12).range( 0.25, 0.5 ) );
	sig = [ sig[0] + sig[1] + sig[2] ];
	sig = LeakDC.ar(sig);
	sig;
});
).play;

Ndef(\dubba).play(0)

///////////////////////////////////// PLANTA 2

MFdef(\p2).add(\post, { |msg| msg[1].postln });
MFdef(\p2).disable(\post);
MFdef(\p2).enable(\post);
(
var arr = 8000!60;
var index = 0;
var menor = 7800;
var maior = 8200;
var average = 8000;
q.v2 = 8000;
Tdef( \buffer, {
    loop {
        if( index < arr.size, {
            arr[index] =q. v2;
        // "valor: % \n".postf(arr[index]);
            index = index + 1;
        }, {
            menor = arr[arr.minIndex];
            // "\nMENOR: % \n".postf(menor);
            maior = arr[arr.maxIndex];
            // "\nMAIOR: % \n".postf(maior);
            average = arr.sum / arr.size;
            index = 0;
        });
        wait(0.065);
    }
}).play;
q.p2Off = q.p2Off ? Bus.control(s);
q.p2Moff = q.p2Moff ? Bus.control(s);
q.p2Normal = q.p2Normal ? Bus.control(s);
q.p2ZeroX = q.p2ZeroX ? Bus.control(s);

MFdef(\p2).add(\handleValues, {|msg|
    q.v2 = msg[1];
    q.p2Off.setSynchronous(q.v2 - 8000);
    q.p2Moff.setSynchronous(q.v2 - average.lag(5) );
    q.vNormal.setSynchronous( q.v2.linlin( menor.lag(5), maior.lag(5), 0, 1 ) );
    q.p2ZeroX.setSynchronous( q.v2.linlin( menor.lag(5), maior.lag(5), -1, 1 ) );
});
// TODO make trigger (threshold)
MFdef(\p2).enable(\handleValues);

MFdef(\p2).add(\postFixOff, { q.p2Off.getSynchronous.postln});
MFdef(\p2).enable(\postFixOff);
MFdef(\p2).disable(\postFixOff);

MFdef(\p2).add(\postMoveOff, { q.p2Moff.getSynchronous.postln});
MFdef(\p2).enable(\postMoveOff);
MFdef(\p2).disable(\postMoveOff);

MFdef(\p2).add(\postNormal, { q.p2Normal.getSynchronous.round(0.01).postln});
MFdef(\p2).enable(\postNormal);
MFdef(\p2).disable(\postNormal);

MFdef(\p2).add(\postZeroX, { q.p2ZeroX.getSynchronous.round(0.01).postln});
MFdef(\p2).enable(\postZeroX);
MFdef(\p2).disable(\postZeroX);

MFdef(\p2).add(\postMin, { menor.postln});
MFdef(\p2).enable(\postMin);
MFdef(\p2).disable(\postMin);

MFdef(\p2).add(\postMax, { menor.postln});
MFdef(\p2).enable(\postMax);
MFdef(\p2).disable(\postMax);
)

(
p.tunel = {|  amp = 0.274, downsamp = 4, modFQ = 2, att = 0.01, rel = 0.1 |
	var son, env, noise, bitReduc, down, pan, initPhase;
    var t_trig  = Trig1.kr( /*q.vNormal.kr.lag(1) */ Changed.kr( q.p2Moff.kr ) ); 
    var freq = ( q.p2Moff.kr.abs + LFNoise2.kr(0.125).exprange(55, LFNoise2.kr(0.2).exprange(110, 200)) * 2 );// ;.linexp(1,35,44,440) ;
	son = ( Saw.ar(freq * 0.5, mul: 0.5) + BlitB3Square.ar(freq, mul: 0.25) + LFPulse.ar(freq * 2, mul: 0.025) ).softclip * 1.5;
    env = EnvGen.ar( Env.asr(att, 1, rel, [4, -4] ), Impulse.kr(1), doneAction: 0 );
    son = RLPF.ar(son, freq);
    noise = PinkNoise.ar(0.15);
    bitReduc = ( son + noise ).round(0.5 ** 8);
    down = Latch.ar(bitReduc, Impulse.ar(SampleRate.ir / downsamp.max(2)));
    initPhase = {1.rrand(0.01)};
    pan = LFTri.ar(modFQ, [initPhase, 1 - initPhase ]);
    son = son.ring3(down) + son * 0.5;
    // son = son * pan * env * amp;
    Mix.ar(son) * amp
}
)
p.tunel.play(1)
//////////////////////////////////////////////////////////////////////



(
Ndef(\say1,{
	var freq = q.vNormal.kr.linexp(0,1,58.3,116.6).lag(\slide.kr(0.2));
    var fdev = { LFNoise2.kr(0.01, 0.1, 1) };
    var vow = Vowel(\o, \bass) * ( \dist.kr(0.05) + LFCub.kr(0.001, 0, 0.02, 0.025) ).lag(0.2);
    var lfmod = { LFNoise2.ar( XLine.kr(1, 6, 1.5), XLine.kr( 0.001, 1, 0.66, 4), 0.501 ) };
    var form = Formants.ar(	[ freq * fdev.reciprocal, freq * fdev ] + lfmod!2, vow,	ampMods: 0.25 );
	var bal = Balance2.ar( form[0][0], form[1][1] );
	var sig = Fold.ar( bal, ( -1 * LFCub.kr(0.01, 0).range(0.1, 2) ), \fold.kr(1) );

	// var trig = Trig1.kr( HPZ1.kr( \threshold.kr(0.15) >= ~pulse1.kr ) );
	var trig = Trig1.kr( Impulse.kr(0.25 + q.vZeroCross.kr.fold2(0.5) ) );
    var tfreq = ZeroCrossing.kr( q.vZeroCross.kr.fold2(0.25) ).lag(1);
    var frecip = ( tfreq + 0.001 ).reciprocal.clip(0.001, 1);
    var relT = LFCub.kr( 0.0013, 1.5pi, frecip, frecip ); // sinusoidal slowly change env release time
    var textr = LFSaw.ar( [900, 820] ).sum * WhiteNoise.ar(1, 0.6);
    var textrT = DelayN.kr( trig, 1, frecip * 0.33 );
    var fb = LocalIn.ar(2).sum;
	var env = Env.perc( 0.001, relT ).ar(gate: trig);
    sig = sig * env;
    fb = DelayC.ar( fb, 1, 0.49 );
    fb = AllpassC.ar( fb, 1, 0.5, 10 );
    fb = LPF.ar( fb, 800, mul:SinOsc.ar(1/8).range(-25, -35).dbamp );
    textr = HPF.ar( textr, LFNoise2.kr(0.05).exprange( 5588, 11175 ) );
    textr = textr * Env( [0, 1, 0], [36/8, 0.0001], 4 ).ar( gate:textrT );
    textr = textr * -40.dbamp;
    textr = Compander.ar( textr, sig, 0.25, 1/1.25, 1/50, 0.01, 0.17 );
    textr = BBandPass.ar( textr, LFNoise2.kr(0.05).exprange(494, 5920), 0.25.lag(0.2) );
    sig = sig + (textr * 0.25);
    Compander.ar(sig, sig, 0.1, 1, 0.125, 0.01, 0.2)
    // LocalOut.ar(sig);
    // Splay(sig) * \amp.kr(0.2);
});
);

(
Ndef(\say2,{
	// var freq = \freq.kr( 77.8, \slide.kr(0.1) );
	var freq = q.vNormal.kr.linexp( 0,1, 77.8, 155.6 ).lag(\slide.kr(0.2));
    var lfmod = { LFNoise2.ar( XLine.kr( 1, 7, 1.5 ), XLine.kr( 0.001, 1, 0.66, 4), 0.501 )};
    var fdev = { LFNoise2.kr( 0.1, 0.01, 1 ) };
    var vow = Vowel(\e, \tenor) * \dist.kr(0.1);
    var form = Formants.ar(	[ freq * fdev.reciprocal, freq * fdev ] + lfmod!2, vow,	ampMods: 0.25 );
	var bal = Balance2.ar( form[0][0], form[1][1] );
    var sig = Fold.ar( bal, (-1 * LFCub.kr(0.01, 0.5pi).range(0.1, 2)), \fold.kr(2) );

    var trig = HPZ1.kr( \threshold.kr(0.15) >=  q.vZeroCross.kr );
    var tfreq = ZeroCrossing.kr( q.vZeroCross.kr.lag(0.2) ).lag(1);
    var frecip = ( tfreq + 0.0001 ).reciprocal.clip(0,1);
    var relT = LFCub.kr( 0.0013, 1.5pi, frecip, frecip );
    var textrT = DelayN.kr( PulseDivider.kr(trig, 2, 0.25), 1, frecip * 0.25 );
    var textr = LFSaw.ar( [880, 698.5] ).sum * WhiteNoise.ar(1, frecip);
    var fb = LocalIn.ar(2).sum;
	var env = Env.perc( 0.001, relT ).ar(gate: trig);
    sig = sig * env;
    fb = DelayC.ar(fb, 1, 0.3675);
    fb = AllpassC.ar(fb, 1, 0.5, 7.5);
    fb = LPF.ar( fb, 800, mul:SinOsc.ar(1/8).range(-25, -35).dbamp );
    textr = HPF.ar( textr, LFNoise2.kr(0.07).exprange(7459, 14917) );
    textr = textr * Env( [0, 1, 0], [tfreq * 3, 0.0001], 4 ).ar(gate:textrT);
    textr = textr * -40.dbamp;
    textr = Compander.ar( textr, sig, 0.25, 1/1.25, 1/50, 0.01, 0.17 );
    textr = BBandPass.ar( textr, LFNoise2.kr(0.05).exprange(2794, 5588), 0.25.lag(0.2) );
    sig = sig + (textr * 0.25);
    Compander.ar(sig, sig, 0.05, 1, 0.25, 0.01, 0.2)
    // LocalOut.ar(sig);
    // Mix(sig) * \amp.kr(0.2);
});
);

(
Ndef(\say3,{
	var freq = \freq.kr( 103.8, \slide.kr(0.1) );
	var freq = q.vNormal.kr.linexp( 0,1, 103.8, 207.3 ).lag(\slide.kr(0.2));
    var lfmod = { LFNoise2.ar( XLine.kr(1, 6, 1.5), XLine.kr( 0.001, 1, 0.66, 4), 0.501 ) };
    var fdev = { LFNoise2.kr(0.1, 0.01, 1) };
    var vow = Vowel(\i, \counterTenor) * \dist.kr(0.12);
    var form = Formants.ar( [ freq * fdev.reciprocal, freq * fdev ] + lfmod!2, vow, ampMods: 0.25 );
    var bal = Balance2.ar( form[0][0], form[1][1] );
    var sig = Fold.ar( bal, (-1 * LFCub.kr(0.01, pi).range(0.1, 2)), \fold.kr(1) );

    var trig = HPZ1.kr( \threshold.kr(0.15) >=  ~pulse3.kr );
    var tfreq = ZeroCrossing.kr(~pulse3.kr.lag(0.2)).lag(1);
    var frecip = ( tfreq + 0.0001 ).reciprocal.clip(0.0001, 1);
    var fb = LocalIn.ar(2).sum;
    var relT = LFCub.kr( 0.0013, 1.5pi, frecip, frecip * 0.8 );
	var env = Env.perc( 0.001, relT ).ar(gate: trig);
    var textrT = DelayN.kr( PulseDivider.kr(trig, 3, 0.5), 1, frecip * 0.2 );
    var textr = LFSaw.ar( [698.5, 523.2]).sum * WhiteNoise.ar(1, frecip);
    fb = DelayC.ar(fb, 1, 0.275625 );
    fb = AllpassC.ar(fb, 1, frecip, freq * 4);
    fb = LPF.ar(fb, 800, mul:SinOsc.ar(frecip).range(-25, -35).dbamp);
    sig = sig * env;
    textr = HPF.ar( textr, LFNoise2.kr(0.07).exprange(5587.5, 11175) );
    textr = textr * Env( [0, 1, 0], [freq*3, 0.0001], 4 ).ar(gate:textrT);
    textr = textr * -40.dbamp;
    textr = Compander.ar( textr, sig, 0.25, 1/1.25, 1/50, 0.01, 0.17 );
    textr = BBandPass.ar(textr, LFNoise2.kr(0.06).exprange(2793.826, 3729.31), 0.25.lag(0.2));
    sig = sig + (textr * 0.25);
    // LocalOut.ar(sig);
	// Mix(sig) * \amp.kr(0.2);
    Compander.ar(sig, sig, 0.05, 1, 0.25, 0.01, 0.2)
});
);

(
p.simples = { | freq = 220, lagt = 0.1, width = 0.5, amp = 0.3, gate = 1, sustain = 1, pan = 1, att = 0.01, dec = 0.5, sus = 0.5, rel = 0.5 |
    var sig, env;
    sig = VarSaw.ar([freq + (q.vNormal.kr * 100), freq * 1.01].lag2(lagt), 0, width, amp);
    sig = SoftClipAmp.ar(sig, 32,  mul: amp );
    env = Env.adsr(att, dec, sus, rel).kr( doneAction: Done.freeSelf, gate: Pulse.kr(1), timeScale: sustain);
    sig = sig * env * amp;
    // sig = BMoog.ar(sig, 220,  q: 0.2,  mode: 0.0,  saturation: 0.95,  mul: 1.0,  add: 0.0);
    sig
    // PanAz.ar(sig, pan)
};
p.simples.play;
)





b = Bus.control;
a={ var ta = q.vNormal.kr.poll; ta}.play(s, b) //you won't hear this if you only have two channels
b.scope

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





