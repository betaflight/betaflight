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
                    // looking ok so far
                    model.html('TRI');
                    
                    process_directions('YAW', 5, 0);
                    
                    process_servos('Yaw Servo', '', SERVO_CONFIG[5], false);
                    
                    servos = [5];
                    break;
                case 4: // BI
                    // looking ok so far
                    model.html('BI');
                    
                    process_directions('L YAW', 4, 1);
                    process_directions('R YAW', 5, 1);
                    process_directions('L NICK', 4, 0);
                    process_directions('R NICK', 5, 0);
                    
                    process_servos('Left Servo', '', SERVO_CONFIG[4], false);
                    process_servos('Right Servo', '', SERVO_CONFIG[5], false);
                    
                    servos = [4, 5];
                    break;
                case 5: // Gimbal
                    // Gimbal doesn't seem to be supported for now
                    model.html('Doesn\'t support servos');
                    
                    /*
                    model.html('Gimbal');
                    
                    process_servos('Pitch Servo', '', SERVO_CONFIG[0]);
                    process_servos('Roll Servo', '', SERVO_CONFIG[1]);
                    
                    servos = [0, 1];
                    */
                    break;
                case 8: // Flying Wing
                    // looking ok so far
                    model.html('Flying Wing');
                    
                    process_directions('L ROLL', 3, 1);
                    process_directions('R ROLL', 4, 1);
                    process_directions('L NICK', 3, 0);
                    process_directions('R NICK', 4, 0);
                    
                    process_servos('Left Wing', '', SERVO_CONFIG[3], false);
                    process_servos('Right Wing', '', SERVO_CONFIG[4], false);
                    
                    servos = [3, 4];
                    break;
                case 14: // Airplane
                    // not implemented yet (rates in %)
                    model.html('Doesn\'t support servos');
                    /*
                    model.html('Airplane');
                    
                    // rate
                    process_servos('Wing 1', '', SERVO_CONFIG[3]);
                    process_servos('Wing 2', '', SERVO_CONFIG[4]);
                    process_servos('Rudd', '', SERVO_CONFIG[5]);
                    process_servos('Elev', '', SERVO_CONFIG[6]);
                    
                    servos = [2, 3, 4, 5, 6];
                    */
                    break;
                case 20: // Dualcopter
                    // looking ok so far
                    model.html('Dualcopter');
                    
                    process_directions('PITCH', 4, 0);
                    process_directions('ROLL', 5, 0);
                    
                    process_servos('Roll', '', SERVO_CONFIG[5], false);
                    process_servos('Nick', '', SERVO_CONFIG[4], false);
                    
                    servos = [4, 5];
                    break;
                case 21: // Singlecopter
                    // looking ok so far
                    model.html('Singlecopter');
                    
                    process_servos('Right', 'R YAW', SERVO_CONFIG[3], true);
                    process_servos('Left', 'L YAW', SERVO_CONFIG[4], true);
                    process_servos('Front', 'F YAW', SERVO_CONFIG[5], true);
                    process_servos('Rear', 'YAW', SERVO_CONFIG[6], true);
                    
                    servos = [3, 4, 5, 6];
                    break;
                default:
                    model.html('Doesn\'t support servos');
            }
        });
    });

    // UI hooks
    $('a.update').click(function() {
        // update bitfields
        $('div.tab-servos table.directions tr:not(".main")').each(function() {
            var info = $('select', this).data();
            info = info.info; // move info one level higher
            var val = parseInt($('select', this).val());
            
            // in this stage we need to know which bitfield and which bitposition needs to be flipped
            if (val) SERVO_CONFIG[info.obj].rate = bit_set(SERVO_CONFIG[info.obj].rate, info.bitpos);
            else SERVO_CONFIG[info.obj].rate = bit_clear(SERVO_CONFIG[info.obj].rate, info.bitpos);
        });
        
        // update the rest
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

            // update rate if direction fields exist
            if ($('.direction input', this).length) {
                if ($('.direction input:first', this).is(':checked')) SERVO_CONFIG[servos[0] + itter].rate = bit_set(SERVO_CONFIG[servos[0] + itter].rate, 0);
                else SERVO_CONFIG[servos[0] + itter].rate = bit_clear(SERVO_CONFIG[servos[0] + itter].rate, 0);
                
                if ($('.direction input:last', this).is(':checked')) SERVO_CONFIG[servos[0] + itter].rate = bit_set(SERVO_CONFIG[servos[0] + itter].rate, 1);
                else SERVO_CONFIG[servos[0] + itter].rate = bit_clear(SERVO_CONFIG[servos[0] + itter].rate, 1);
            }
            
            itter++;
        });
        
        // send settings over to mcu
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


function process_directions(name, obj, bitpos) {
    var val;
    
    $('div.tab-servos table.directions').append('\
        <tr>\
            <td class="name" style="text-align: center">' + name + '</td>\
            <td class="direction" style="text-align: right">\
                <select name="direction">\
                    <option value="0">Normal</option>\
                    <option value="1">Reverse</option>\
                </select>\
            </td>\
        </tr>\
    ');
    
    if (bit_check(SERVO_CONFIG[obj].rate, bitpos)) val = 1;
    else val = 0;

    $('div.tab-servos table.directions tr:last select').val(val);
    $('div.tab-servos table.directions tr:last select').data('info', {'obj': obj, 'bitpos': bitpos});
}

function process_servos(name, alternate, obj, directions) {    
    $('div.tab-servos table.fields').append('\
        <tr> \
            <td style="text-align: center">' + name + '</td>\
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
    ');    
    
    if (obj.middle <= 7) {
        $('div.tab-servos table.fields tr:last td.middle input').prop('disabled', true);
        $('div.tab-servos table.fields tr:last td.channel').find('input').eq(obj.middle).prop('checked', true);
    }
    
    if (directions == true) {
        $('div.tab-servos table.fields tr:last td.direction input:first').prop('checked', bit_check(obj.rate, 0));
        $('div.tab-servos table.fields tr:last td.direction input:last').prop('checked', bit_check(obj.rate, 1));
        
        // store additional data
        //$('div.tab-servos table.fields tr:last td.direction').data('info', {});
    } else if (directions == 2) {
        // reserved for rate
    } else {
        $('div.tab-servos table.fields tr:last td.direction').html('');
    }
    
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