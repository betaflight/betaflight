'use strict';

// generate mixer
var mixerList = [
    {name: 'Tricopter',        model: 'tricopter',  image: 'tri'},
    {name: 'Quad +',           model: 'quad_x',     image: 'quad_p'},
    {name: 'Quad X',           model: 'quad_x',     image: 'quad_x'},
    {name: 'Bicopter',         model: 'custom',     image: 'bicopter'},
    {name: 'Gimbal',           model: 'custom',     image: 'custom'},
    {name: 'Y6',               model: 'y6',         image: 'y6'},
    {name: 'Hex +',            model: 'hex_plus',   image: 'hex_p'},
    {name: 'Flying Wing',      model: 'custom',     image: 'flying_wing'},
    {name: 'Y4',               model: 'y4',         image: 'y4'},
    {name: 'Hex X',            model: 'hex_x',      image: 'hex_x'},
    {name: 'Octo X8',          model: 'custom',     image: 'octo_x8'},
    {name: 'Octo Flat +',      model: 'custom',     image: 'octo_flat_p'},
    {name: 'Octo Flat X',      model: 'custom',     image: 'octo_flat_x'},
    {name: 'Airplane',         model: 'custom',     image: 'airplane'},
    {name: 'Heli 120',         model: 'custom',     image: 'custom'},
    {name: 'Heli 90',          model: 'custom',     image: 'custom'},
    {name: 'V-tail Quad',      model: 'quad_vtail', image: 'vtail_quad'},
    {name: 'Hex H',            model: 'custom',     image: 'custom'},
    {name: 'PPM to SERVO',     model: 'custom',     image: 'custom'},
    {name: 'Dualcopter',       model: 'custom',     image: 'custom'},
    {name: 'Singlecopter',     model: 'custom',     image: 'custom'},
    {name: 'A-tail Quad',      model: 'quad_atail', image: 'atail_quad'},
    {name: 'Custom',           model: 'custom',     image: 'custom'},
    {name: 'Custom Airplane',  model: 'custom',     image: 'custom'},
    {name: 'Custom Tricopter', model: 'custom',     image: 'custom'}
];


// 3D model
var Model = function (wrapper, canvas) {
    var useWebGLRenderer = this.canUseWebGLRenderer();

    this.wrapper = wrapper;
    this.canvas = canvas;

    if (useWebGLRenderer) {
        this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas[0], alpha: true, antialias: true });
    } else {
        this.renderer = new THREE.CanvasRenderer({ canvas: this.canvas[0], alpha: true });
    }

    // initialize render size for current canvas size
    this.renderer.setSize(this.wrapper.width() * 2, this.wrapper.height() * 2);

    // load the model including materials
    var model_file = useWebGLRenderer ? mixerList[BF_CONFIG.mixerConfiguration - 1].model : 'fallback';

    // Temporary workaround for 'custom' model until akfreak's custom model is merged.
    if (model_file == 'custom') { model_file = 'fallback'; }

    // setup scene
    this.scene = new THREE.Scene();

    // modelWrapper adds an extra axis of rotation to avoid gimbal lock with the euler angles
    this.modelWrapper = new THREE.Object3D();

    // stationary camera
    this.camera = new THREE.PerspectiveCamera(60, this.wrapper.width() / this.wrapper.height(), 1, 10000);

    // move camera away from the model
    this.camera.position.z = 125;

    // some light
    var light = new THREE.AmbientLight(0x404040);
    var light2 = new THREE.DirectionalLight(new THREE.Color(1, 1, 1), 1.5);
    light2.position.set(0, 1, 0);

    // add camera, model, light to the foreground scene
    this.scene.add(light);
    this.scene.add(light2);
    this.scene.add(this.camera);
    this.scene.add(this.modelWrapper);

    // Load model file, add to scene and render it
    this.loadJSON(model_file, (function (model) {
        this.model = model;

        this.modelWrapper.add(model);
        this.scene.add(this.modelWrapper);

        this.render();
    }).bind(this));
};

Model.prototype.loadJSON = function (model_file, callback) {
    var loader = new THREE.JSONLoader();

    loader.load('./resources/models/' + model_file + '.json', function (geometry, materials) {
        var modelMaterial = new THREE.MeshFaceMaterial(materials),
            model         = new THREE.Mesh(geometry, modelMaterial);

        model.scale.set(15, 15, 15);

        callback(model);
    });
};

Model.prototype.canUseWebGLRenderer = function () {
    // webgl capability detector
    // it would seem the webgl "enabling" through advanced settings will be ignored in the future
    // and webgl will be supported if gpu supports it by default (canary 40.0.2175.0), keep an eye on this one
    var detector_canvas = document.createElement('canvas');

    return window.WebGLRenderingContext && (detector_canvas.getContext('webgl') || detector_canvas.getContext('experimental-webgl'))
};

Model.prototype.rotateTo = function (x, y, z) {
    if (!this.model) { return; }

    this.model.rotation.x = x;
    this.modelWrapper.rotation.y = y;
    this.model.rotation.z = z;

    this.render();
};

Model.prototype.rotateBy = function (x, y, z) {
    if (!this.model) { return; }

    this.model.rotateX(x);
    this.model.rotateY(y);
    this.model.rotateZ(z);

    this.render();
};

Model.prototype.render = function () {
    if (!this.model) { return; }

    // draw
    this.renderer.render(this.scene, this.camera);
};

// handle canvas resize
Model.prototype.resize = function () {
    this.renderer.setSize(this.wrapper.width() * 2, this.wrapper.height() * 2);

    this.camera.aspect = this.wrapper.width() / this.wrapper.height();
    this.camera.updateProjectionMatrix();

    this.render();
};
