'use strict';

function update_model(multiType) {

    // Display multiType and motor diagram (if such exist)
    var str = '';
    switch (multiType) {
        case 1: // TRI
            str = 'TRI';
            $('.modelMixDiagram').attr('src', './images/motor_order/tri.svg').addClass('modelMixTri');
            break;
        case 2: // QUAD +
            str = 'Quad +';
            $('.modelMixDiagram').attr('src', './images/motor_order/quadp.svg').addClass('modelMixQuadP');
            break;
        case 3: // QUAD X
            str = 'Quad X';
            $('.modelMixDiagram').attr('src', './images/motor_order/quadx.svg').addClass('modelMixQuadX');
            break;
        case 4: // BI
            str = 'BI';
            break;
        case 5: // GIMBAL
            str = 'Gimbal';
            break;
        case 6: // Y6
            str = 'Y6';
            $('.modelMixDiagram').attr('src', './images/motor_order/y6.svg').addClass('modelMixY6');
            break;
        case 7: // HEX 6
            str = 'HEX 6';
            $('.modelMixDiagram').attr('src', './images/motor_order/hex6p.svg').addClass('modelMixHex6P');
            break;
        case 8: // FLYING_WING
            str = 'Flying Wing';
            break;
        case 9: // Y4
            str = 'Y4';
            $('.modelMixDiagram').attr('src', './images/motor_order/y4.svg').addClass('modelMixY4');
            break;
        case 10: // HEX6 X
            str = 'HEX6 X';
            $('.modelMixDiagram').attr('src', './images/motor_order/hex6x.svg').addClass('modelMixHex6X');
            break;
        case 11: // OCTO X8
        case 12:
        case 13:
            str = 'OCTO X8';
            $('.modelMixDiagram').attr('src', './images/motor_order/octox.svg').addClass('modelMixOctoX');
            break;
        case 14: // AIRPLANE
            str = 'Airplane';
            $('.modelMixDiagram').attr('src', './images/motor_order/airplane.svg').addClass('modelMixAirplane');
            break;
        case 15: // Heli 120
            str = 'Heli 120';
            break;
        case 16: // Heli 90
            str = 'Heli 90';
            break;
        case 17: // Vtail
            str = 'Vtail';
            $('.modelMixDiagram').attr('src', './images/motor_order/vtail.svg').addClass('modelMixVtail');
            break;
        case 18: // HEX6 H
            str = 'HEX6 H';
            $('.modelMixDiagram').attr("src", './images/motor_order/custom.svg').addClass('modelMixCustom');
            break;
        case 19: // PPM to SERVO
            str = 'PPM to SERVO';
            $('.modelMixDiagram').attr("src", './images/motor_order/custom.svg').addClass('modelMixCustom');
            break;
        case 20: // Dualcopter
            str = 'Dualcopter';
            $('.modelMixDiagram').attr("src", './images/motor_order/custom.svg').addClass('modelMixCustom');
            break;
        case 21: // Singlecopter
            str = 'Singlecopter';
            $('.modelMixDiagram').attr("src", './images/motor_order/custom.svg').addClass('modelMixCustom');
            break;
    }

    $('span.model').text(chrome.i18n.getMessage('initialSetupModel', [str]));
};
