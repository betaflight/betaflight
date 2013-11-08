function tab_initialize_servos() {
    ga_tracker.sendAppView('Servos');
    
    // request current Servos Config
    send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT);
    send_message(MSP_codes.MSP_SERVO_CONF, MSP_codes.MSP_SERVO_CONF);
    
    var model = $('div.tab-servos strong.model');
    var servos = [];
    
    setTimeout(function() {
        switch (CONFIG.multiType) {
            case 1: // TRI
                model.html('TRI');
                
                process_servos('Rear', SERVO_CONFIG[5], 5);
                
                servos = [5];
                break;
            case 4: // BI
                model.html('BI');
                
                process_servos('Left', SERVO_CONFIG[4], 4);
                process_servos('Right', SERVO_CONFIG[5], 5);
                
                servos = [4, 5];
                break;
            case 5: // Gimbal
                model.html('Gimbal');
                
                process_servos('Pitch Servo', SERVO_CONFIG[0], 0);
                process_servos('Roll Servo', SERVO_CONFIG[1], 1);
                
                servos = [0, 1];
                break;
            case 8: // Flying Wing
                model.html('Flying Wing');
                
                process_servos('Left', SERVO_CONFIG[3], 3);
                process_servos('Right', SERVO_CONFIG[4], 4);
                
                servos = [3, 4];
                break;
            case 14: // Airplane
                model.html('Airplane');
                
                process_servos('Wing 1', SERVO_CONFIG[3], 3);
                process_servos('Wing 2', SERVO_CONFIG[4], 4);
                process_servos('Rudd', SERVO_CONFIG[5], 5);
                process_servos('Elev', SERVO_CONFIG[6], 6);
                process_servos('Thro', SERVO_CONFIG[7], 7);
                
                servos = [2, 3, 4, 5, 6, 7];
                break;
            case 20: // Dualcopter
                model.html('Dualcopter');
                
                process_servos('Pitch', SERVO_CONFIG[4], 4);
                process_servos('Roll', SERVO_CONFIG[5], 5);
                process_servos('M 1', SERVO_CONFIG[6], 6);
                process_servos('M 0', SERVO_CONFIG[7], 7);
                
                servos = [4, 5, 6, 7];
                break;
            case 21: // Singlecopter
                model.html('Singlecopter');
                
                process_servos('Right', SERVO_CONFIG[3], 3);
                process_servos('Left', SERVO_CONFIG[4], 4);
                process_servos('Front', SERVO_CONFIG[5], 5);
                process_servos('Rear', SERVO_CONFIG[6], 6);
                process_servos('Motor', SERVO_CONFIG[7], 7);
                
                servos = [3, 4, 5, 6, 7];
                break;
        }
    }, 50);
}

function process_servos(name, obj, pos) {    
    $('div.tab-servos table.fields').append('\
        <tr> \
            <td style="text-align: center;">' + name + '</td>\
            <td class="middle"><input type="number" min="1000" max="2000" value="' + obj.middle +'" /></td>\
            <td class="min"><input type="number" min="1000" max="2000" value="' + obj.min +'" /></td>\
            <td class="max"><input type="number" min="1000" max="2000" value="' + obj.max +'" /></td>\
            <td class="channel">\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
            </td>\
            <td class="direction"></td>\
        </tr> \
    '
    );    
    
    if (obj.middle <= 7) {
        $('div.tab-servos table.fields tr:last td.channel').find('input').eq(obj.middle).prop('checked', true);
    }
    
    // UI hooks
    $('div.tab-servos table.fields tr:last td.channel').find('input').click(function() {
        if($(this).is(':checked')) {
            $(this).parent().parent().parent().find('td.middle input').prop('disabled', true);
            $('div.tab-servos table.fields tr:last td.channel').find('input').not($(this)).prop('checked', false);
        } else {
            $(this).parent().parent().parent().find('td.middle input').prop('disabled', false);
        }
    });
}