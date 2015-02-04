'use strict';

var BOARD_DEFINITIONS = [
    {
        name: "CC3D",
        identifier: "CC3D"
    }, {
        name: "ChebuzzF3",
        identifier: "CHF3"
    }, {
        name: "CJMCU",
        identifier: "CJM1"
    }, {
        name: "EUSTM32F103RB",
        identifier: "EUF1"
    }, {
        name: "Naze/Flip32+",
        identifier: "AFNA"
    }, {
        name: "Naze32Pro",
        identifier: "AFF3"
    }, {
        name: "Olimexino",
        identifier: "OLI1"
    }, {
        name: "Port103R",
        identifier: "103R"
    }, {
        name: "Sparky",
        identifier: "SPKY"
    }, {
        name: "STM32F3Discovery",
        identifier: "SDF3"
    }, {
        name: "SP Racing F3",
        identifier: "SRF3"
    }
   }
];

var DEFAULT_BOARD_DEFINITION = {
    name: "Unknown",
    identifier: "????"
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

