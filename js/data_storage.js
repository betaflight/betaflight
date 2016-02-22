'use strict';

var CONFIGURATOR = {
    'releaseDate': 1456164608903, // new Date().getTime() - Mon Feb 22 2016 19:10:05 GMT+0100
    
     // all versions are specified and compared using semantic versioning http://semver.org/
    'apiVersionAccepted': '1.2.1',
    'backupRestoreMinApiVersionAccepted': '1.5.0',
    'pidControllerChangeMinApiVersion': '1.5.0',
    'backupFileMinVersionAccepted': '0.55.0', // chrome.runtime.getManifest().version is stored as string, so does this one
    
    'connectionValid': false,
    'connectionValidCliOnly': false,
    'cliActive': false,
    'cliValid': false
};
