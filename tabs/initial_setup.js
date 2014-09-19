'use strict';

TABS.initial_setup = {
    yaw_fix: 0.0
};

TABS.initial_setup.initialize = function (callback) {
    var self = this;
    GUI.active_tab_ref = this;
    GUI.active_tab = 'initial_setup';
    googleAnalytics.sendAppView('Setup');

    function load_ident() {
        MSP.send_message(MSP_codes.MSP_IDENT, false, false, load_config);
    }

    function load_config() {
        if (bit_check(CONFIG.capability, 30)) {
            MSP.send_message(MSP_codes.MSP_CONFIG, false, false, load_misc_data);
        } else {
            load_misc_data();
        }
    }

    function load_misc_data() {
        MSP.send_message(MSP_codes.MSP_MISC, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/initial_setup.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_ACC_TRIM, false, false, load_ident);

    function process_html() {
        // translate to user-selected language
        localize();

        // if CAP_BASEFLIGHT_CONFIG (30)
        if (bit_check(CONFIG.capability, 30)) {
            // current stuff, this will become default when the compatibility period ends
            $('.CAP_BASEFLIGHT_CONFIG').show();

            // initialize 3D
            self.initialize3D(false);

            // set heading in interactive block
            $('span.heading').text(chrome.i18n.getMessage('initialSetupheading', [0]));
        } else {
            // old stuff
            $('.COMPATIBILITY').show();

            // initialize 3D
            self.initialize3D(true);

            // Fill in misc stuff
            $('input[name="mincellvoltage"]').val(MISC.vbatmincellvoltage);
            $('input[name="maxcellvoltage"]').val(MISC.vbatmaxcellvoltage);
            $('input[name="voltagescale"]').val(MISC.vbatscale);

            $('input[name="minthrottle"]').val(MISC.minthrottle);
            $('input[name="maxthrottle"]').val(MISC.maxthrottle);
            $('input[name="failsafe_throttle"]').val(MISC.failsafe_throttle);
            $('input[name="mincommand"]').val(MISC.mincommand);

            $('input[name="mag_declination"]').val(MISC.mag_declination / 10);

            // Fill in the accel trimms from CONFIG object
            $('input[name="pitch"]').val(CONFIG.accelerometerTrims[0]);
            $('input[name="roll"]').val(CONFIG.accelerometerTrims[1]);

            // Display multiType and motor diagram (if such exist)
            var str;
            switch (CONFIG.multiType) {
                case 1: // TRI
                    str = 'TRI';
                    $('.modelMixDiagram').attr('src', './images/motor_order/tri.svg').addClass('modelMixTri');
                    break;
                case 2: // QUAD +
                    str = 'Quad +';
                    $('.modelMixDiagram').attr('src', './images/motor_order/quad_p.svg').addClass('modelMixQuadP');
                    break;
                case 3: // QUAD X
                    str = 'Quad X';
                    $('.modelMixDiagram').attr('src', './images/motor_order/quad_x.svg').addClass('modelMixQuadX');
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
                    $('.modelMixDiagram').attr('src', './images/motor_order/hex_p.svg').addClass('modelMixHex6P');
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
                    $('.modelMixDiagram').attr('src', './images/motor_order/hex_x.svg').addClass('modelMixHex6X');
                    break;
                case 11: // OCTO X8
                case 12:
                case 13:
                    str = 'OCTO X8';
                    $('.modelMixDiagram').attr('src', './images/motor_order/octo_flat_x.svg').addClass('modelMixOctoX');
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
                    $('.modelMixDiagram').attr('src', './images/motor_order/vtail_quad.svg').addClass('modelMixVtail');
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
                case 22: // Custom
                    str = 'Custom';
                    $('.modelMixDiagram').attr("src", './images/motor_order/custom.svg').addClass('modelMixCustom');
                    break;
            }

            $('span.model').text(chrome.i18n.getMessage('initialSetupModel', [str]));

            // Heading
            $('span.heading').text(chrome.i18n.getMessage('initialSetupheading', [0]));

            $('a.update').click(function () {
                CONFIG.accelerometerTrims[0] = parseInt($('input[name="pitch"]').val());
                CONFIG.accelerometerTrims[1] = parseInt($('input[name="roll"]').val());

                MISC.vbatmincellvoltage = parseFloat($('input[name="mincellvoltage"]').val()) * 10;
                MISC.vbatmaxcellvoltage = parseFloat($('input[name="maxcellvoltage"]').val()) * 10;
                MISC.vbatscale = parseInt($('input[name="voltagescale"]').val());

                MISC.minthrottle = parseInt($('input[name="minthrottle"]').val());
                MISC.maxthrottle = parseInt($('input[name="maxthrottle"]').val());
                MISC.failsafe_throttle = parseInt($('input[name="failsafe_throttle"]').val());
                MISC.mincommand = parseInt($('input[name="mincommand"]').val());

                MISC.mag_declination = parseFloat($('input[name="mag_declination"]').val()) * 10;

                function save_to_eeprom() {
                    MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                        GUI.log(chrome.i18n.getMessage('initialSetupEepromSaved'));
                    });
                }

                // Send over the new trims
                MSP.send_message(MSP_codes.MSP_SET_ACC_TRIM, MSP.crunch(MSP_codes.MSP_SET_ACC_TRIM));

                // Send over new misc
                MSP.send_message(MSP_codes.MSP_SET_MISC, MSP.crunch(MSP_codes.MSP_SET_MISC), false, save_to_eeprom);
            });
        }

        // check if we have magnetometer
        if (!bit_check(CONFIG.activeSensors, 2)) {
            $('a.calibrateMag').addClass('disabled');
        }

        // UI Hooks
        $('a.calibrateAccel').click(function () {
            var self = $(this);

            if (!self.hasClass('calibrating')) {
                self.addClass('calibrating');

                // During this period MCU won't be able to process any serial commands because its locked in a for/while loop
                // until this operation finishes, sending more commands through data_poll() will result in serial buffer overflow
                GUI.interval_pause('initial_setup_data_pull');
                MSP.send_message(MSP_codes.MSP_ACC_CALIBRATION, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('initialSetupAccelCalibStarted'));
                });

                GUI.timeout_add('button_reset', function () {
                    GUI.interval_resume('initial_setup_data_pull');

                    GUI.log(chrome.i18n.getMessage('initialSetupAccelCalibEnded'));

                    self.removeClass('calibrating');
                }, 2000);
            }
        });

        $('a.calibrateMag').click(function () {
            var self = $(this);

            if (!self.hasClass('calibrating') && !self.hasClass('disabled')) {
                self.addClass('calibrating');

                MSP.send_message(MSP_codes.MSP_MAG_CALIBRATION, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('initialSetupMagCalibStarted'));
                });

                GUI.timeout_add('button_reset', function () {
                    GUI.log(chrome.i18n.getMessage('initialSetupMagCalibEnded'));
                    self.removeClass('calibrating');
                }, 30000);
            }
        });

        $('a.resetSettings').click(function() {
            MSP.send_message(MSP_codes.MSP_RESET_CONF, false, false, function () {
                GUI.log(chrome.i18n.getMessage('initialSetupSettingsRestored'));

                GUI.tab_switch_cleanup(function() {
                    TABS.initial_setup.initialize();
                });
            });
        });

        // display current yaw fix value (important during tab re-initialization)
        $('div#interactive_block > a.reset').text(chrome.i18n.getMessage('initialSetupButtonResetZaxisValue', [self.yaw_fix]));

        // reset yaw button hook
        $('div#interactive_block > a.reset').click(function () {
            self.yaw_fix = SENSOR_DATA.kinematics[2] * - 1.0;
            $(this).text(chrome.i18n.getMessage('initialSetupButtonResetZaxisValue', [self.yaw_fix]));

            console.log('YAW reset to 0 deg, fix: ' + self.yaw_fix + ' deg');
        });

        $('#content .backup').click(configuration_backup);

        $('#content .restore').click(configuration_restore);

        // data pulling functions used inside interval timer
        // this stuff will be reworked when compatibility period ends, to make the pulling more efficient
        function get_analog_data() {
            MSP.send_message(MSP_codes.MSP_ANALOG, false, false, get_gps_data);
        }

        function get_gps_data() {
            MSP.send_message(MSP_codes.MSP_RAW_GPS, false, false, get_attitude_data);
        }

        function get_attitude_data() {
            MSP.send_message(MSP_codes.MSP_ATTITUDE, false, false, update_ui);
        }

        // in future update selectors will be moved outside to specific variables to increase performance
        function update_ui() {
            // Update heading
            $('span.heading').text(chrome.i18n.getMessage('initialSetupheading', [SENSOR_DATA.kinematics[2]]));

            // Update voltage indicator
            $('.bat-voltage').text(chrome.i18n.getMessage('initialSetupBatteryValue', [ANALOG.voltage]));
            $('.bat-mah-drawn').text(chrome.i18n.getMessage('initialSetupBatteryMahValue', [ANALOG.mAhdrawn]));
            $('.bat-mah-drawing').text(chrome.i18n.getMessage('initialSetupBatteryAValue', [ANALOG.amperage.toFixed(2)]));
            $('.rssi').text(chrome.i18n.getMessage('initialSetupRSSIValue', [((ANALOG.rssi / 1023) * 100).toFixed(0)]));

            // Update gps
            $('.gpsFix').html((GPS_DATA.fix) ? chrome.i18n.getMessage('gpsFixTrue') : chrome.i18n.getMessage('gpsFixFalse'));
            $('.gpsSats').text(GPS_DATA.numSat);
            $('.gpsLat').text((GPS_DATA.lat / 10000000).toFixed(4) + ' deg');
            $('.gpsLon').text((GPS_DATA.lon / 10000000).toFixed(4) + ' deg');

            // Update 3D
            self.render3D();
        }

        GUI.interval_add('initial_setup_data_pull', get_analog_data, 50, true);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull () {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }
};

TABS.initial_setup.initialize3D = function (compatibility) {
    var self = this;

    if (compatibility) {
        var canvas = $('.COMPATIBILITY #canvas');
        var wrapper = $('.COMPATIBILITY #canvas_wrapper');
    } else {
        var canvas = $('.CAP_BASEFLIGHT_CONFIG #canvas');
        var wrapper = $('.CAP_BASEFLIGHT_CONFIG #canvas_wrapper');
    }

    var camera = new THREE.PerspectiveCamera(50, wrapper.width() / wrapper.height(), 1, 10000);

    var renderer;
    if (GUI.operating_system != 'Linux' || GUI.operating_system != 'CrOS') {
        renderer = new THREE.WebGLRenderer({canvas: canvas.get(0), alpha: true, antialias: true});
    } else {
        renderer = new THREE.CanvasRenderer({canvas: canvas.get(0), alpha: true});
    }
    var scene = new THREE.Scene();

    // some light
    var light = new THREE.DirectionalLight(new THREE.Color(1,1,1), 0.5);
    light.position.set(0, 1, 0);

    // flying brick
    var modelWrapper = new THREE.Object3D();
    var geometry = new THREE.BoxGeometry(150, 80, 300);
    var materialArray = [ // overdraw helps remove wireframe when using CanvasRenderer
        new THREE.MeshLambertMaterial({color: 0xff3333, emissive: 0x962020, overdraw: true}), // right
        new THREE.MeshLambertMaterial({color: 0xff8800, emissive: 0xa45a06, overdraw: true}), // left
        new THREE.MeshLambertMaterial({color: 0xffff33, emissive: 0x9a9a21, overdraw: true}), // top
        new THREE.MeshLambertMaterial({color: 0x33ff33, emissive: 0x1f901f, overdraw: true}), // bottom
        new THREE.MeshLambertMaterial({color: 0x3333ff, emissive: 0x212192, overdraw: true}), // back
        new THREE.MeshLambertMaterial({color: 0x8833ff, emissive: 0x5620a2, overdraw: true}), // front
    ];
    var materials = new THREE.MeshFaceMaterial(materialArray);
    var model = new THREE.Mesh(geometry, materials);

    // initialize render size for current canvas size
    renderer.setSize(wrapper.width(), wrapper.height());

    // move camera away from the model
    camera.position.z = 500;

    // modelWrapper just adds an extra axis of rotation to avoid gimbal lock withe euler angles
    modelWrapper.add(model);

    // add camera, model, light to the foreground scene
    scene.add(camera);
    scene.add(modelWrapper);
    scene.add(light);


    this.render3D = function () {
        // compute the changes
        model.rotation.x = (SENSOR_DATA.kinematics[1] * -1.0) * 0.017453292519943295;
        modelWrapper.rotation.y = ((SENSOR_DATA.kinematics[2] * -1.0) - self.yaw_fix) * 0.017453292519943295;
        model.rotation.z = (SENSOR_DATA.kinematics[0] * -1.0) * 0.017453292519943295;

        // draw
        renderer.render(scene, camera);
    };

    // handle canvas resize
    $(window).resize(function () {
        renderer.setSize(wrapper.width(), wrapper.height());
        camera.aspect = wrapper.width() / wrapper.height();
        camera.updateProjectionMatrix();

        self.render3D();
    });
};

TABS.initial_setup.cleanup = function (callback) {
    $(window).unbind('resize');

    if (callback) callback();
};