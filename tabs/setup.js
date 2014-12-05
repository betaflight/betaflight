'use strict';

TABS.setup = {
    yaw_fix: 0.0
};

TABS.setup.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'setup') {
        GUI.active_tab = 'setup';
        googleAnalytics.sendAppView('Setup');
    }

    function load_ident() {
        MSP.send_message(MSP_codes.MSP_IDENT, false, false, load_config);
    }

    function load_config() {
        MSP.send_message(MSP_codes.MSP_CONFIG, false, false, load_misc_data);
    }

    function load_misc_data() {
        MSP.send_message(MSP_codes.MSP_MISC, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/setup.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_ACC_TRIM, false, false, load_ident);

    function process_html() {
        // translate to user-selected language
        localize();

        // initialize 3D
        self.initialize3D();

        // set heading in interactive block
        $('span.heading').text(chrome.i18n.getMessage('initialSetupheading', [0]));

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
                GUI.interval_pause('setup_data_pull');
                MSP.send_message(MSP_codes.MSP_ACC_CALIBRATION, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('initialSetupAccelCalibStarted'));
                });

                GUI.timeout_add('button_reset', function () {
                    GUI.interval_resume('setup_data_pull');

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

        $('a.resetSettings').click(function () {
            MSP.send_message(MSP_codes.MSP_RESET_CONF, false, false, function () {
                GUI.log(chrome.i18n.getMessage('initialSetupSettingsRestored'));

                GUI.tab_switch_cleanup(function () {
                    TABS.setup.initialize();
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

        $('#content .backup').click(function () {
            configuration_backup(function () {
                GUI.log(chrome.i18n.getMessage('initialSetupBackupSuccess'));
                googleAnalytics.sendEvent('Configuration', 'Backup', 'true');
            });
        });

        $('#content .restore').click(function () {
            configuration_restore(function () {
                GUI.log(chrome.i18n.getMessage('initialSetupRestoreSuccess'));
                googleAnalytics.sendEvent('Configuration', 'Restore', 'true');

                // get latest settings
                TABS.setup.initialize();
            });
        });

        // cached elements
        var bat_voltage_e = $('.bat-voltage'),
            bat_mah_drawn_e = $('.bat-mah-drawn'),
            bat_mah_drawing_e = $('.bat-mah-drawing'),
            rssi_e = $('.rssi'),
            gpsFix_e = $('.gpsFix'),
            gpsSats_e = $('.gpsSats'),
            gpsLat_e = $('.gpsLat'),
            gpsLon_e = $('.gpsLon'),
            heading_e = $('span.heading');

        function get_slow_data() {
            MSP.send_message(MSP_codes.MSP_ANALOG, false, false, function () {
                bat_voltage_e.text(chrome.i18n.getMessage('initialSetupBatteryValue', [ANALOG.voltage]));
                bat_mah_drawn_e.text(chrome.i18n.getMessage('initialSetupBatteryMahValue', [ANALOG.mAhdrawn]));
                bat_mah_drawing_e.text(chrome.i18n.getMessage('initialSetupBatteryAValue', [ANALOG.amperage.toFixed(2)]));
                rssi_e.text(chrome.i18n.getMessage('initialSetupRSSIValue', [((ANALOG.rssi / 1023) * 100).toFixed(0)]));
            });

            MSP.send_message(MSP_codes.MSP_RAW_GPS, false, false, function () {
                gpsFix_e.html((GPS_DATA.fix) ? chrome.i18n.getMessage('gpsFixTrue') : chrome.i18n.getMessage('gpsFixFalse'));
                gpsSats_e.text(GPS_DATA.numSat);
                gpsLat_e.text((GPS_DATA.lat / 10000000).toFixed(4) + ' deg');
                gpsLon_e.text((GPS_DATA.lon / 10000000).toFixed(4) + ' deg');
            });

            MSP.send_message(MSP_codes.MSP_STATUS);
        }

        function get_fast_data() {
            MSP.send_message(MSP_codes.MSP_ATTITUDE, false, false, function () {
                heading_e.text(chrome.i18n.getMessage('initialSetupheading', [SENSOR_DATA.kinematics[2]]));
                self.render3D();
            });
        }

        GUI.interval_add('setup_data_pull_fast', get_fast_data, 33, true); // 30 fps
        GUI.interval_add('setup_data_pull_slow', get_slow_data, 250, true); // 4 fps

        if (callback) callback();
    }
};

TABS.setup.initialize3D = function (compatibility) {
    var self = this,
        loader, canvas, wrapper, renderer, camera, scene, light, light2, modelWrapper, model, model_file,
        fallback = false;

    canvas = $('.CAP_BASEFLIGHT_CONFIG #canvas');
    wrapper = $('.CAP_BASEFLIGHT_CONFIG #canvas_wrapper');

    // webgl capability detector
    // it would seem the webgl "enabling" through advanced settings will be ignored in the future
    // and webgl will be supported if gpu supports it by default (canary 40.0.2175.0), keep an eye on this one
    var detector_canvas = document.createElement('canvas');
    if (window.WebGLRenderingContext && (detector_canvas.getContext('webgl') || detector_canvas.getContext('experimental-webgl'))) {
        renderer = new THREE.WebGLRenderer({canvas: canvas.get(0), alpha: true, antialias: true});
    } else {
        renderer = new THREE.CanvasRenderer({canvas: canvas.get(0), alpha: true});
        fallback = true;
    }

    // modelWrapper just adds an extra axis of rotation to avoid gimbal lock withe euler angles
    modelWrapper = new THREE.Object3D()

    // load the model including materials
    var models = [
        'tricopter',
        'quad_x',
        'quad_x',
        'quad_x',
        'quad_x',
        'y6',
        'hex_plus',
        'quad_x',
        'y4',
        'hex_x',
        'quad_x',
        'quad_x',
        'quad_x',
        'quad_x',
        'quad_x',
        'quad_x',
        'quad_vtail',
        'quad_x',
        'quad_x',
        'quad_x',
        'quad_x',
        'quad_atail',
        'quad_x'
    ];

    if (!fallback) {
        model_file = models[CONFIG.multiType - 1];
    } else {
        model_file = 'fallback';
    }

    loader = new THREE.JSONLoader();
    loader.load('./resources/models/' + model_file + '.json', function (geometry, materials) {
        if (!fallback) {
            model = new THREE.Mesh(geometry, new THREE.MeshFaceMaterial(materials));
        } else {
            materials = THREE.ImageUtils.loadTexture('./resources/textures/fallback_texture.png');
            model = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({map: materials, overdraw: true}));
        }

        model.scale.set(10, 10, 10);

        modelWrapper.add(model);
        scene.add(modelWrapper);
    });

    // stacionary camera
    camera = new THREE.PerspectiveCamera(50, wrapper.width() / wrapper.height(), 1, 10000);

    // setup scene
    scene = new THREE.Scene();

    // some light
    light = new THREE.AmbientLight(0x404040);
    light2 = new THREE.DirectionalLight(new THREE.Color(1, 1, 1), 1.5);
    light2.position.set(0, 1, 0);

    // initialize render size for current canvas size
    renderer.setSize(wrapper.width(), wrapper.height());

    // move camera away from the model
    camera.position.z = 125;

    // add camera, model, light to the foreground scene
    scene.add(light);
    scene.add(light2);
    scene.add(camera);
    scene.add(modelWrapper);

    this.render3D = function () {
        // compute the changes
        model.rotation.x = (SENSOR_DATA.kinematics[1] * -1.0) * 0.017453292519943295;
        modelWrapper.rotation.y = ((SENSOR_DATA.kinematics[2] * -1.0) - self.yaw_fix) * 0.017453292519943295;
        model.rotation.z = (SENSOR_DATA.kinematics[0] * -1.0) * 0.017453292519943295;

        // draw
        renderer.render(scene, camera);
    };

    // handle canvas resize
    this.resize3D = function () {
        renderer.setSize(wrapper.width(), wrapper.height());
        camera.aspect = wrapper.width() / wrapper.height();
        camera.updateProjectionMatrix();

        self.render3D();
    };

    $(window).on('resize', this.resize3D);
};

TABS.setup.cleanup = function (callback) {
    $(window).off('resize', this.resize3D);

    if (callback) callback();
};
