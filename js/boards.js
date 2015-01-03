'use strict';

var BOARD_DEFINITIONS = [
    {
        name: "CC3D",
        identifier: "CC3D",
        serialPortCount: 3
    }, {
        name: "ChebuzzF3",
        identifier: "CHF3",
        serialPortCount: 3
    }, {
        name: "CJMCU",
        identifier: "CJM1",
        serialPortCount: 2
    }, {
        name: "EUSTM32F103RB",
        identifier: "EUF1",
        serialPortCount: 4
    }, {
        name: "Naze/Flip32+",
        identifier: "AFNA",
        serialPortCount: 4
    }, {
        name: "Naze32Pro",
        identifier: "AFF3",
        serialPortCount: 3
    }, {
        name: "Olimexino",
        identifier: "OLI1",
        serialPortCount: 4
    }, {
        name: "Port103R",
        identifier: "103R",
        serialPortCount: 4
    }, {
        name: "Sparky",
        identifier: "SPKY",
        serialPortCount: 4
    }, {
        name: "STM32F3Discovery",
        identifier: "SDF3",
        serialPortCount: 3
    }
];

var DEFAULT_BOARD_DEFINITION = {
    name: "Unknown",
    identifier: "????",
    serialPortCount: 1
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

