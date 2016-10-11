'use strict';

// This list has been extracted from the firmware source with:
// grep TARGET_BOARD_IDENTIFIER src/main/target/*/target.h | sed -n "s/^src\/main\/target\/\([^\/]*\).*#define TARGET_BOARD_IDENTIFIER \"\([^\"]*\).*$/{name: '\1', identifier: '\2', vcp: false},/p" | sort
// and then manually setting vcp to true for boards that use VCP

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
        name: "Olimexino",
        identifier: "OLI1",
        vcp: false
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
    }, {
        name: "Lux Race",
        identifier: "LUX",
        vcp: true
    }, {
        name: "KISS FC",
        identifier: "KISS",
        vcp: true
    }, {
        name: "Singularity",
        identifier: "SING",
        vcp: true 
    }, {
        name: "VRRACE",
        identifier: "VRRA",
        vcp: true
    }, {
        name: "Colibri",
        identifier: "COLI",
        vcp: true
    }, {
        name: "Sparky 2",
        identifier: "SPK2",
        vcp: true
    }, {
        name: "Air Hero F3",
        identifier: "AIR3",
        vcp: false
    }, {
        name: "AIORACERF3",
        identifier: "ARF3",
        vcp: true
    }, {
        name: "ALIENFLIGHTF4",
        identifier: "AFF4",
        vcp: true
    }, {
        name: "F4BY",
        identifier: "F4BY",
        vcp: true
    }, {
        name: "X_RACERSPI",
        identifier: "XRC3",
        vcp: false
    }, {
        name: "AIR32",
        identifier: "AR32",
        vcp: true
    }, {
        name: "FURYF3",
        identifier: "FYF3",
        vcp: true
    }, {
        name: "ALIENFLIGHTF1",
        identifier: "AFF1",
        vcp: false
    }, {
        name: "PIKOBLX",
        identifier: "PIKO",
        vcp: true
    }, {
        name: "OMNIBUSF4",
        identifier: "OBF4",
        vcp: true
    }, {
        name: "REVONANO",
        identifier: "REVN",
        vcp: true
    }, {
        name: "DOGE",
        identifier: "DOGE",
        vcp: true
    }, {
        name: "BLUEJAYF4",
        identifier: "BJF4",
        vcp: true
    }, {
        name: "SIRINFPV",
        identifier: "SIRF",
        vcp: false
    }, {
        name: "OMNIBUS",
        identifier: "OMNI",
        vcp: true
    }, {
        name: "FURYF4",
        identifier: "FYF4",
        vcp: true
    }, {
        name: "ZCOREF3",
        identifier: "ZCF3",
        vcp: false
    }, {
        name: "MICROSCISKY",
        identifier: "MSKY",
        vcp: false
    }, {
        name: "RMDO",
        identifier: "RMDO",
        vcp: false
    }, {
        name: "REVO",
        identifier: "REVO",
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
