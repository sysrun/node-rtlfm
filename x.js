'use strict';

var rtlsdr = require('./');


rtlsdr.getDevices(function (err, devices) {
  if (err) {
    console.error(err);
  } else {
    //console.log(devices);
    var dev = devices[0];
    dev.open(function (er) {
      if (er) {
        console.log(er);
      } else {
        console.log(dev.test());
      }
      dev.on('pcmdata', function (d) {
        console.log(d.length);
      });
    });
  }

});
