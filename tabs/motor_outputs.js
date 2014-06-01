function tab_initialize_motor_outputs() {
    ga_tracker.sendAppView('Motor Outputs Page');
    GUI.active_tab = 'motor_outputs';

    MSP.send_message(MSP_codes.MSP_MISC, false, false, get_motor_data);

    function get_motor_data() {
        MSP.send_message(MSP_codes.MSP_MOTOR, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/motor_outputs.html", process_html);
    }

    function process_html() {
        // translate to user-selected language
        localize();

        // if CAP_DYNBALANCE is true
        if (bit_check(CONFIG.capability, 2)) {
            $('div.motor_testing').show();
        }

        var number_of_valid_outputs = (MOTOR_DATA.indexOf(0) > -1) ? MOTOR_DATA.indexOf(0) : 8;

        $('input.min').val(MISC.mincommand);
        $('input.max').val(MISC.maxthrottle);


        $('div.sliders input').prop('min', MISC.mincommand);
        $('div.sliders input').prop('max', MISC.maxthrottle);
        $('div.sliders input').val(MISC.mincommand);
        $('div.values li:not(:last)').html(MISC.mincommand);

        // UI hooks
        $('div.sliders input:not(.master)').on('input', function() {
            var index = $(this).index();

            $('div.values li').eq(index).html($(this).val());

            // send data to mcu
            var buffer_out = [];

            for (var i = 0; i < 8; i++) {
                var val = parseInt($('div.sliders input').eq(i).val());

                buffer_out.push(lowByte(val));
                buffer_out.push(highByte(val));
            }

            MSP.send_message(MSP_codes.MSP_SET_MOTOR, buffer_out);
        });

        $('div.sliders input.master').on('input', function() {
            var val = $(this).val();

            $('div.sliders input:not(:disabled, :last)').val(val);
            $('div.values li:not(:last)').slice(0, number_of_valid_outputs).html(val);
            $('div.sliders input:not(:last):first').trigger('input');
        });

        $('div.notice input[type="checkbox"]').change(function() {
            if ($(this).is(':checked')) {
                $('div.sliders input').slice(0, number_of_valid_outputs).prop('disabled', false);

                // unlock master slider
                $('div.sliders input:last').prop('disabled', false);
            } else {
                // disable sliders / min max
                $('div.sliders input').prop('disabled', true);

                // change all values to default
                $('div.sliders input').val(1000);
                $('div.values li:not(:last)').html(1000);

                // trigger change event so values are sent to mcu
                $('div.sliders input').trigger('input');
            }
        });

        // data pulling functions used inside interval timer
        function get_motor_data() {
            MSP.send_message(MSP_codes.MSP_MOTOR, false, false, get_servo_data);
        }

        function get_servo_data() {
            MSP.send_message(MSP_codes.MSP_SERVO, false, false, update_ui);
        }

        function update_ui() {
            var block_height = $('div.m-block:first').height();

            for (var i = 0; i < MOTOR_DATA.length; i++) {
                var data = MOTOR_DATA[i] - 1000;
                var margin_top = block_height - (data * (block_height / 1000));
                var height = (data * (block_height / 1000));
                var color = parseInt(data * 0.256);

                $('.motor-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
            }

            for (var i = 0; i < SERVO_DATA.length; i++) {
                var data = SERVO_DATA[i] - 1000;
                var margin_top = block_height - (data * (block_height / 1000));
                var height = (data * (block_height / 1000));
                var color = parseInt(data * 0.256);

                $('.servo-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
            }
        }

        // enable Motor data pulling
        GUI.interval_add('motor_pull', get_motor_data, 50, true);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);
    }
}