'use strict';

var BOARD_DEFINITIONS = [
    {
        name: "CC3D",
        identifier: "CC3D",
        vcp: true
    }, {
        name: "ChebuzzF3",
        identifier: "CHF3",
        vcp: false
    }, {
        name: "CJMCU",
        identifier: "CJM1",
        vcp: false
    }, {
        name: "EUSTM32F103RB",
        identifier: "EUF1",
        vcp: false
    }, {
        name: "Naze/Flip32+",
        identifier: "AFNA",
        vcp: false
    }, {
        name: "Naze32Pro",
        identifier: "AFF3",
        vcp: false
    }, {
        name: "Olimexino",
        identifier: "OLI1"
    }, {
        name: "Port103R",
        identifier: "103R",
        vcp: false
    }, {
        name: "Sparky",
        identifier: "SPKY",
        vcp: true
    }, {
        name: "STM32F3Discovery",
        identifier: "SDF3",
        vcp: true
    }, {
        name: "Colibri Race",
        identifier: "CLBR",
        vcp: true
    }, {
        name: "SP Racing F3",
        identifier: "SRF3",
        vcp: false
    }, {
        name: "SP Racing F3 Mini",
        identifier: "SRFM",
        vcp: true
    }, {
        name: "SP Racing F3 EVO",
        identifier: "SPEV",
        vcp: true
    }, {
        name: "Alienflight F3",
        identifier: "AFF3",
        vcp: true
    }, {
        name: "ImmersionRC Fusion F3",
        identifier: "IFF3",
        vcp: false
    }, {
        name: "MotoLab",
        identifier: "MOTO",
        vcp: true
    }
];

var DEFAULT_BOARD_DEFINITION = {
    name: "Unknown",
    identifier: "????",
    vcp: false
};

var BOARD = {
    
};

BOARD.find_board_definition = function (identifier) {
    for (var i = 0; i < BOARD_DEFINITIONS.length; i++) {
        var candidate = BOARD_DEFINITIONS[i];
        
        if (candidate.identifier == identifier) {
            return candidate;
        }
    }
    return DEFAULT_BOARD_DEFINITION;
};
