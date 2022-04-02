// data log
q.dataFile2 = CSVFileWriter( "bromelia_%.csv".format(Date.getDate.format("%Y_%m_%d-%H_%M_%S")), "w", stamp: false);
MFdef(\p2).add(\dataLog, { |msg|
    q.dataFile2.writeLine( msg );
});
Routine.run( { do({ MFdef(\p1).enable(\dataLog); wait(60*30); MFdef(\p1).disable(\dataLog); }) } )
MFdef(\p0).remove(\dataLog);
