'use strict';

var rtlsdr = require('./');


rtlsdr.getDevices(function (err, devices) {
  if (err) {
    console.error(err);
  } else {
    //console.log(devices);
    var dev = devices[0];
    console.log(dev);
    console.log(dev.test());
  }

});
