function tab_initialize_servos() {
    ga_tracker.sendAppView('Servos');
    GUI.active_tab = 'servos';

    var model = $('div.tab-servos strong.model');
    var servos = [];
    
    // request current Servos Config
    send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT, false, function() {
        send_message(MSP_codes.MSP_SERVO_CONF, MSP_codes.MSP_SERVO_CONF, false, function() {
            // drop previous table
            $('div.tab-servos table.fields tr:not(:first)').remove();
            
            switch (CONFIG.multiType) {
                case 1: // TRI
                    // Broken
                    model.html('TRI');
                    
                    process_servos('Yaw Servo', '', SERVO_CONFIG[5]);
                    
                    servos = [5];
                    break;
                case 4: // BI
                    // Broken
                    model.html('BI');
                    
                    process_servos('Left', '', SERVO_CONFIG[4]);
                    process_servos('Right', '', SERVO_CONFIG[5]);
                    
                    servos = [4, 5];
                    break;
                case 5: // Gimbal
                    // Broken
                    model.html('Gimbal');
                    
                    process_servos('Pitch Servo', '', SERVO_CONFIG[0]);
                    process_servos('Roll Servo', '', SERVO_CONFIG[1]);
                    
                    servos = [0, 1];
                    break;
                case 8: // Flying Wing
                    // Broken
                    model.html('Flying Wing');
                    
                    process_servos('L ROLL', 'R ROLL', SERVO_CONFIG[3]);
                    process_servos('L NICK', 'R NICK', SERVO_CONFIG[4]);
                    
                    servos = [3, 4];
                    break;
                case 14: // Airplane
                    // Broken
                    model.html('Airplane');
                    
                    // rate
                    process_servos('Wing 1', '', SERVO_CONFIG[3]);
                    process_servos('Wing 2', '', SERVO_CONFIG[4]);
                    process_servos('Rudd', '', SERVO_CONFIG[5]);
                    process_servos('Elev', '', SERVO_CONFIG[6]);
                    process_servos('Thro', '', SERVO_CONFIG[7]);
                    
                    servos = [2, 3, 4, 5, 6, 7];
                    break;
                case 20: // Dualcopter
                    // Broken
                    // Gyro / Acc direction: 4
                    // Roll: 5, Nick: 4
                    model.html('Dualcopter');
                    
                    process_servos('PITCH', 'ROLL', SERVO_CONFIG[4]);
                    process_servos('Roll', '', SERVO_CONFIG[5]);
                    
                    servos = [4, 5];
                    break;
                case 21: // Singlecopter
                    // Broken
                    model.html('Singlecopter');
                    
                    process_servos('Right', 'R YAW', SERVO_CONFIG[3]);
                    process_servos('Left', 'L YAW', SERVO_CONFIG[4]);
                    process_servos('Front', 'F YAW', SERVO_CONFIG[5]);
                    process_servos('Rear', 'YAW', SERVO_CONFIG[6]);
                    
                    servos = [3, 4, 5, 6];
                    break;
                default:
                    model.html('Doesn\'t support servos');
            }
        });
    });

    // UI hooks
    $('a.update').click(function() {
        // update objects
        var itter = 0;
        $('div.tab-servos table.fields tr:not(".main")').each(function() {
            if ($('.middle input', this).is(':disabled')) {
                var val = $('.channel input:checked', this).index();
                
                SERVO_CONFIG[servos[0] + itter].middle = parseInt(val);
            } else {
                SERVO_CONFIG[servos[0] + itter].middle = parseInt($('.middle input', this).val());
            }
            
            SERVO_CONFIG[servos[0] + itter].min = parseInt($('.min input', this).val());
            SERVO_CONFIG[servos[0] + itter].max = parseInt($('.max input', this).val());
            
            var rate = 0;
            if ($('.direction input:first', this).is(':checked')) {
                rate |= 0x01;
            }
            
            if ($('.direction input:last', this).is(':checked')) {
                rate |= 0x02;
            }
            
            SERVO_CONFIG[servos[0] + itter].rate = rate;

            itter++;
        });
        
        
        var buffer_out = [];
        
        var needle = 0;
        for (var i = 0; i < SERVO_CONFIG.length; i++) {
            buffer_out[needle++] = lowByte(SERVO_CONFIG[i].min);
            buffer_out[needle++] = highByte(SERVO_CONFIG[i].min);
            
            buffer_out[needle++] = lowByte(SERVO_CONFIG[i].max);
            buffer_out[needle++] = highByte(SERVO_CONFIG[i].max);
            
            buffer_out[needle++] = lowByte(SERVO_CONFIG[i].middle);
            buffer_out[needle++] = highByte(SERVO_CONFIG[i].middle);
            
            buffer_out[needle++] = lowByte(SERVO_CONFIG[i].rate);
        }
        
        send_message(MSP_codes.MSP_SET_SERVO_CONF, buffer_out);
        
        // Save changes to EEPROM
        send_message(MSP_codes.MSP_EEPROM_WRITE, MSP_codes.MSP_EEPROM_WRITE);
    });
}

function process_servos(name, alternate, obj) {    
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
            <td class="direction">\
                <input class="first" type="checkbox"/><span class="name">' + name + '</span>\
                <input class="second" type="checkbox"/><span class="alternate">' + alternate + '</span>\
            </td>\
        </tr> \
    '
    );    
    
    if (obj.middle <= 7) {
        $('div.tab-servos table.fields tr:last td.middle input').prop('disabled', true);
        $('div.tab-servos table.fields tr:last td.channel').find('input').eq(obj.middle).prop('checked', true);
    }
    
    $('div.tab-servos table.fields tr:last td.direction input:first').prop('checked', bit_check(obj.rate, 0));
    $('div.tab-servos table.fields tr:last td.direction input:last').prop('checked', bit_check(obj.rate, 1));
    
    // UI hooks
    $('div.tab-servos table.fields tr:last td.channel').find('input').click(function() {
        if($(this).is(':checked')) {
            $(this).parent().parent().find('td.middle input').prop('disabled', true);
            $(this).parent().find('input').not($(this)).prop('checked', false);
        } else {
            $(this).parent().parent().find('td.middle input').prop('disabled', false).val(1500);
        }
    });
}