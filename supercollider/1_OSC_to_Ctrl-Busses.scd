(
var arr = 8100!64;
var index = 0;
var j = 1;
var menor = 8000;
var maior = 8500;
var average = 8250;
q.v0 = 8500;
Tdef( \buffer, {
    loop {
        if( index < arr.size, {
            arr[index] =q.v0;
        // "valor: % \n".postf(arr[index]);
            index = index + 1;
        }, {
            menor = arr[arr.minIndex];
            // "\nMENOR: % \n".postf(menor);
            maior = arr[arr.maxIndex];
            // "\nMAIOR: % \n \n".postf(maior);
            average = arr.sum / arr.size;
            index = 0;
        });
        // "test".postln;
        wait(0.1);
    }
}).play;
q.vFixOffset = q.vFixOffset ? Bus.control(s);
q.vMoveOffset = q.vMoveOffset ? Bus.control(s);
q.vNormal = q.vNormal ? Bus.control(s);
q.vZeroCross = q.vZeroCross ? Bus.control(s);

MFdef(\p0).add(\handleValues, {|msg|
    q.v0 = msg[1].asInteger;
    // q.v0.postln;
    q.vFixOffset.setSynchronous( q.v0 - 8100);
    q.vMoveOffset.setSynchronous(q.v0 - average.lag(1) );
    q.vNormal.setSynchronous( q.v0.linlin( menor.lag(0.2), maior.lag(0.2), 0, 1 ) );
    q.vZeroCross.setSynchronous( q.v0.linlin( menor.lag(1), maior.lag(1), -1, 1 ) );
});
// TODO make trigger (threshold)
MFdef(\p0).enable(\handleValues);

MFdef(\p0).add(\postFixOff, { q.vFixOffset.getSynchronous.round(0.001).postln});
MFdef(\p0).enable(\postFixOff);
MFdef(\p0).disable(\postFixOff);

MFdef(\p0).add(\postMoveOff, { q.vMoveOffset.getSynchronous.round(0.01).postln});
MFdef(\p0).enable(\postMoveOff);
MFdef(\p0).disable(\postMoveOff);

MFdef(\p0).add(\postNormal, { q.vNormal.getSynchronous.round(0.001).postln});
MFdef(\p0).enable(\postNormal);
MFdef(\p0).disable(\postNormal);

MFdef(\p0).add(\postZeroX, { q.vZeroCross.getSynchronous.round(0.001).postln});
MFdef(\p0).enable(\postZeroX);
MFdef(\p0).disable(\postZeroX);
)

Tdef(\buffer).clear;
