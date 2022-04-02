// play plant data
n = NetAddr("127.0.0.1", 57120); // 57120 is sclang default port
(
var bromeliaDataLog = CSVFilePlayer( "/home/skmecs/Trabalho/banda-planta/supercollider/bromelia_2022_03_05-16_01_09.csv", true);
var lineNum = 456206;
Tdef(\playBromeliaData, {
//     var msg;
    loop {
        ( lineNum -1).do{ |i|
            var msg = bromeliaDataLog.readAtLine(i%lineNum);
            msg = msg.pop;
            // msg.postln;
            n.sendMsg("/suculenta", msg);
            0.04.wait;
        };
        bromeliaDataLog.reset;
        "volta ao comeco".postln;
    }
});
)
Tdef(\playBromeliaData).play;
Tdef(\playBromeliaData).pause;

