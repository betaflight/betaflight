'use strict';

var CONFIGURATOR = {
    'releaseDate': 1448724175378, // new Date().getTime() - Sat Nov 28 2015 15:22:51 GMT+0000 (GMT Standard Time)
    
     // all versions are specified and compared using semantic versioning http://semver.org/
    'apiVersionAccepted': '1.2.0',
    'backupRestoreMinApiVersionAccepted': '1.5.0',
    'pidControllerChangeMinApiVersion': '1.5.0',
    'backupFileMinVersionAccepted': '0.55.0', // chrome.runtime.getManifest().version is stored as string, so does this one
    
    'connectionValid': false,
    'connectionValidCliOnly': false,
    'cliActive': false,
    'cliValid': false
};
