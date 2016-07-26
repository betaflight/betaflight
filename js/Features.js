'use strict;'

var Features = function (config) {
    var features = [
        {bit: 0, group: 'rxMode', mode: 'group', name: 'RX_PPM'},
        {bit: 1, group: 'batteryVoltage', name: 'VBAT'},
        {bit: 2, group: 'other', name: 'INFLIGHT_ACC_CAL'},
        {bit: 3, group: 'rxMode', mode: 'group', name: 'RX_SERIAL'},
        {bit: 4, group: 'esc', name: 'MOTOR_STOP'},
        {bit: 5, group: 'other', name: 'SERVO_TILT'},
        {bit: 6, group: 'other', name: 'SOFTSERIAL', haveTip: true},
        {bit: 7, group: 'gps', name: 'GPS', haveTip: true},
        {bit: 8, group: 'rxFailsafe', name: 'FAILSAFE'},
        {bit: 9, group: 'other', name: 'SONAR'},
        {bit: 10, group: 'other', name: 'TELEMETRY'},
        {bit: 11, group: 'batteryCurrent', name: 'CURRENT_METER'},
        {bit: 12, group: 'other', name: '3D'},
        {bit: 13, group: 'rxMode', mode: 'group', name: 'RX_PARALLEL_PWM'},
        {bit: 14, group: 'rxMode', mode: 'group', name: 'RX_MSP'},
        {bit: 15, group: 'rssi', name: 'RSSI_ADC'},
        {bit: 16, group: 'other', name: 'LED_STRIP'},
        {bit: 17, group: 'other', name: 'DISPLAY'},
        {bit: 19, group: 'other', name: 'BLACKBOX', haveTip: true}
    ];

    if (semver.gte(config.apiVersion, "1.12.0")) {
        features.push(
            {bit: 20, group: 'other', name: 'CHANNEL_FORWARDING'}
        );
    }

    if (semver.gte(config.apiVersion, "1.16.0")) {
        features.push(
            {bit: 21, group: 'other', name: 'TRANSPONDER', haveTip: true}
        );
    }

    if (config.flightControllerVersion !== '' && semver.gte(config.flightControllerVersion, "2.8.0")) {
         features.push(
            {bit: 22, group: 'other', name: 'AIRMODE'},
            {bit: 23, group: 'pidTuning', name: 'SUPEREXPO_RATES'},
            {bit: 24, group: 'other', name: 'OSD'}
        );
    }

    this._features = features;
    this._featureMask = 0;
}

Features.prototype.getMask = function () {
    return this._featureMask;
}

Features.prototype.setMask = function (featureMask) {
    this._featureMask = featureMask;
}

Features.prototype.isEnabled = function (featureName) {
    var features = this._features;
    var featureMask = this._featureMask;

    for (var i = 0; i < features.length; i++) {
        if (features[i].name === featureName && bit_check(featureMask, features[i].bit)) {
            return true;
        }
    }
    return false;
}

Features.prototype.generateElements = function (featuresElements) {
    var features = this._features;
    var featureMask = this._featureMask;
    var radioGroups = [];

    for (var i = 0; i < features.length; i++) {
        var row_e;

        var feature_tip_html = '';
        if (features[i].haveTip) {
            feature_tip_html = '<div class="helpicon cf_tip" i18n_title="feature' + features[i].name + 'Tip"></div>';
        }

        if (features[i].mode === 'group') {
            row_e = $('<tr><td style="width: 15px;"><input style="width: 13px;" class="feature" id="feature-'
                    + i
                    + '" value="'
                    + features[i].bit
                    + '" title="'
                    + features[i].name
                    + '" type="radio" name="'
                    + features[i].group
                    + '" /></td><td><label for="feature-'
                    + i
                    + '">'
                    + features[i].name
                    + '</label></td><td><span i18n="feature' + features[i].name + '"></span>'
                    + feature_tip_html + '</td></tr>');
            radioGroups.push(features[i].group);
        } else {
            row_e = $('<tr><td><input class="feature toggle" id="feature-'
                    + i
                    + '" name="'
                    + features[i].name
                    + '" title="'
                    + features[i].name
                    + '" type="checkbox"/></td><td><label for="feature-'
                    + i
                    + '">'
                    + features[i].name
                    + '</label></td><td><span i18n="feature' + features[i].name + '"></span>'
                    + feature_tip_html + '</td></tr>');

            var feature_e = row_e.find('input.feature');

            feature_e.prop('checked', bit_check(featureMask, features[i].bit));
            feature_e.data('bit', features[i].bit);
        }

        featuresElements.each(function () {
            if ($(this).hasClass(features[i].group)) {
                $(this).append(row_e);
            }
        });
    }

    for (var i = 0; i < radioGroups.length; i++) {
        var group = radioGroups[i];
        var controlElements = $('input[name="' + group + '"].feature');

        controlElements.each(function() {
            var bit = parseInt($(this).attr('value'));
            var state = bit_check(featureMask, bit);

            $(this).prop('checked', state);
        });
    }
}

Features.prototype.updateData = function (featureElement) {
    switch (featureElement.attr('type')) {
        case 'checkbox':
            var bit = featureElement.data('bit');

            if (featureElement.is(':checked')) {
                this._featureMask = bit_set(this._featureMask, bit);
            } else {
                this._featureMask = bit_clear(this._featureMask, bit);
            }

            break;
        case 'radio':
            var group = featureElement.attr('name');
            var controlElements = $('input[name="' + group + '"]');
            var selectedBit = controlElements.filter(':checked').val();
            var featureMask = this._featureMask;
            controlElements.each(function() {
                var bit = $(this).val();
                if (selectedBit === bit) {
                	featureMask = bit_set(featureMask, bit);
                } else {
                	featureMask = bit_clear(featureMask, bit);
                }

            });
            this._featureMask = featureMask;
            break;
    }
}
