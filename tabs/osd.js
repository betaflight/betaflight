'use strict';

var SYM = SYM || {};
SYM.VOLT = 0x06;
SYM.RSSI = 0x01;
SYM.AH_RIGHT = 0x02;
SYM.AH_LEFT = 0x03;
SYM.THR = 0x04;
SYM.THR1 = 0x05;
SYM.FLY_M = 0x9C;
SYM.ON_M = 0x9B;
SYM.AH_CENTER_LINE = 0x26;
SYM.AH_CENTER_LINE_RIGHT = 0x27;
SYM.AH_CENTER = 0x7E;
SYM.AH_BAR9_0 = 0x80;
SYM.AH_DECORATION = 0x13;
SYM.LOGO = 0xA0;
SYM.AMP = 0x9A;
SYM.MAH = 0x07;

var FONT = FONT || {};

FONT.initData = function() {
  if (FONT.data) {
    return;
  }
  FONT.data = {
    // default font file name
    loaded_font_file: 'default',
    // array of arry of image bytes ready to upload to fc
    characters_bytes: [],
    // array of array of image bits by character
    characters: [],
    // an array of base64 encoded image strings by character
    character_image_urls: []
  }
};

FONT.constants = {
  SIZES: {
    /** NVM ram size for one font char, actual character bytes **/
    MAX_NVM_FONT_CHAR_SIZE: 54,
    /** NVM ram field size for one font char, last 10 bytes dont matter **/
    MAX_NVM_FONT_CHAR_FIELD_SIZE: 64,
    CHAR_HEIGHT: 18,
    CHAR_WIDTH: 12,
    LINE: 30
  },
  COLORS: {
    // black
    0: 'rgba(0, 0, 0, 1)',
    // also the value 3, could yield transparent according to
    // https://www.sparkfun.com/datasheets/BreakoutBoards/MAX7456.pdf
    1: 'rgba(255, 255, 255, 0)',
    // white
    2: 'rgba(255,255,255, 1)'
  }
};

/**
 * Each line is composed of 8 asci 1 or 0, representing 1 bit each for a total of 1 byte per line
 */
FONT.parseMCMFontFile = function(data) {
  var data = data.split("\n");
  // clear local data
  FONT.data.characters.length = 0;
  FONT.data.characters_bytes.length = 0;
  FONT.data.character_image_urls.length = 0;
  // make sure the font file is valid
  if (data.shift().trim() != 'MAX7456') {
    var msg = 'that font file doesnt have the MAX7456 header, giving up';
    console.debug(msg);
    Promise.reject(msg);
  }
  var character_bits = [];
  var character_bytes = [];
  // hexstring is for debugging
  FONT.data.hexstring = [];
  var pushChar = function() {
    FONT.data.characters_bytes.push(character_bytes);
    FONT.data.characters.push(character_bits);
    FONT.draw(FONT.data.characters.length-1);
    //$log.debug('parsed char ', i, ' as ', character);
    character_bits = [];
    character_bytes = [];
  };
  for (var i = 0; i < data.length; i++) {
    var line = data[i];
    // hexstring is for debugging
    FONT.data.hexstring.push('0x' + parseInt(line, 2).toString(16));
    // every 64 bytes (line) is a char, we're counting chars though, which are 2 bits
    if (character_bits.length == FONT.constants.SIZES.MAX_NVM_FONT_CHAR_FIELD_SIZE * (8 / 2)) {
      pushChar()
    }
    for (var y = 0; y < 8; y = y + 2) {
      var v = parseInt(line.slice(y, y+2), 2);
      character_bits.push(v);
    }
    character_bytes.push(parseInt(line, 2));
  }
  // push the last char
  pushChar();
  return FONT.data.characters;
};


FONT.openFontFile = function($preview) {
  return new Promise(function(resolve) {
    chrome.fileSystem.chooseEntry({type: 'openFile', accepts: [{extensions: ['mcm']}]}, function (fileEntry) {
      FONT.data.loaded_font_file = fileEntry.name;
      if (chrome.runtime.lastError) {
          console.error(chrome.runtime.lastError.message);
          return;
      }
      fileEntry.file(function (file) {
        var reader = new FileReader();
        reader.onloadend = function(e) {
          if (e.total != 0 && e.total == e.loaded) {
            FONT.parseMCMFontFile(e.target.result);
            resolve();
          }
          else {
            console.error('could not load whole font file');
          }
        };
        reader.readAsText(file);
      });
    });
  });
};

/**
 * returns a canvas image with the character on it
 */
var drawCanvas = function(charAddress) {
  var canvas = document.createElement('canvas');
  var ctx = canvas.getContext("2d");

  // TODO: do we want to be able to set pixel size? going to try letting the consumer scale the image.
  var pixelSize = pixelSize || 1;
  var width = pixelSize * FONT.constants.SIZES.CHAR_WIDTH;
  var height = pixelSize * FONT.constants.SIZES.CHAR_HEIGHT;

  canvas.width = width;
  canvas.height = height;

  for (var y = 0; y < height; y++) {
    for (var x = 0; x < width; x++) {
      if (!(charAddress in FONT.data.characters)) {
        console.log('charAddress', charAddress, ' is not in ', FONT.data.characters.length);
      }
      var v = FONT.data.characters[charAddress][(y*width)+x];
      ctx.fillStyle = FONT.constants.COLORS[v];
      ctx.fillRect(x, y, pixelSize, pixelSize);
    }
  }
  return canvas;
};

FONT.draw = function(charAddress) {
  var cached = FONT.data.character_image_urls[charAddress];
  if (!cached) {
    cached = FONT.data.character_image_urls[charAddress] = drawCanvas(charAddress).toDataURL('image/png');
  }
  return cached;
};

FONT.msp = {
  encode: function(charAddress) {
    return [charAddress].concat(FONT.data.characters_bytes[charAddress].slice(0,FONT.constants.SIZES.MAX_NVM_FONT_CHAR_SIZE));
  }
};

FONT.upload = function($progress) {
  return Promise.mapSeries(FONT.data.characters, function(data, i) {
    $progress.val((i / FONT.data.characters.length) * 100);
    return MSP.promise(MSPCodes.MSP_OSD_CHAR_WRITE, FONT.msp.encode(i));
  })
  .then(function() {
    return MSP.promise(MSPCodes.MSP_SET_REBOOT);
  });
};

FONT.preview = function($el) {
  $el.empty()
  for (var i = 0; i < SYM.LOGO; i++) {
    var url = FONT.data.character_image_urls[i];
    $el.append('<img src="'+url+'" title="0x'+i.toString(16)+'"></img>');
  }
};

FONT.symbol = function(hexVal) {
  return String.fromCharCode(hexVal);
};

var OSD = OSD || {};

// parsed fc output and output to fc, used by to OSD.msp.encode
OSD.initData = function() {
  OSD.data = {
    video_system: null,
    display_items: [],
    last_positions: {},
    preview: []
  };
};
OSD.initData();

OSD.constants = {
  VIDEO_TYPES: [
    'AUTO',
    'PAL',
    'NTSC'
  ],
  VIDEO_LINES: {
    PAL: 16,
    NTSC: 13
  },
  VIDEO_BUFFER_CHARS: {
    PAL: 480,
    NTSC: 390
  },
  AHISIDEBARWIDTHPOSITION: 7,
  AHISIDEBARHEIGHTPOSITION: 3,
  // order matters, so these are going in an array... pry could iterate the example map instead
  DISPLAY_FIELDS: [
    {
      name: 'MAIN_BATT_VOLTAGE',
      default_position: -29,
      positionable: true,
      preview: FONT.symbol(SYM.VOLT) + '16.8'
    },
    {
      name: 'RSSI_VALUE',
      default_position: -59,
      positionable: true,
      preview: FONT.symbol(SYM.RSSI) + '99'
    },
    {
      name: 'TIMER',
      default_position: -39,
      positionable: true,
      preview: FONT.symbol(SYM.ON_M) + ' 11:11'
    },
    {
      name: 'THROTTLE_POSITION',
      default_position: -9,
      positionable: true,
      preview: FONT.symbol(SYM.THR) + FONT.symbol(SYM.THR1) + ' 69'
    },
    {
      name: 'CPU_LOAD',
      default_position: 26,
      positionable: true,
      preview: '15'
    },
    {
      name: 'VTX_CHANNEL',
      default_position: 1,
      positionable: true,
      preview: 'CH:1'
    },
    {
      name: 'VOLTAGE_WARNING',
      default_position: -80,
      positionable: true,
      preview: 'LOW VOLTAGE'
    },
    {
      name: 'ARMED',
      default_position: -107,
      positionable: true,
      preview: 'ARMED'
    },
    {
      name: 'DIASRMED',
      default_position: -109,
      positionable: true,
      preview: 'DISARMED'
    },
    {
      name: 'ARTIFICIAL_HORIZON',
      default_position: -1,
      positionable: false
    },
    {
      name: 'HORIZON_SIDEBARS',
      default_position: -1,
      positionable: false
    },
    {
      name: 'CURRENT_DRAW',
      default_position: -23,
      positionable: true,
      preview: FONT.symbol(SYM.AMP) + '42.0'
    },
    {
      name: 'MAH_DRAWN',
      default_position: -18,
      positionable: true,
      preview: FONT.symbol(SYM.MAH) + '690'
    },
    {
      name: 'CRAFT_NAME',
      default_position: -77,
      positionable: true,
      preview: '[CRAFT_NAME]'
    }
  ],
};

OSD.updateDisplaySize = function() {
  var video_type = OSD.constants.VIDEO_TYPES[OSD.data.video_system];
  if (video_type == 'AUTO') {
    video_type = 'PAL';
  }
  // compute the size
  OSD.data.display_size = {
    x: 30,
    y: OSD.constants.VIDEO_LINES[video_type],
    total: null
  };
};

OSD.msp = {
  encodeOther: function() {
    return [-1, OSD.data.video_system];
  },
  encode: function(display_item) {
    return [
      display_item.index,
      specificByte(display_item.position, 0),
      specificByte(display_item.position, 1)
    ];
  },
  // Currently only parses MSP_MAX_OSD responses, add a switch on payload.code if more codes are handled
  decode: function(payload) {
    var view = payload.data;
    var d = OSD.data;
    d.compiled_in = view.getUint8(0, 1);
    d.video_system = view.getUint8(1, 1);
    d.display_items = [];
    // start at the offset from the other fields
    for (var i = 2; i < view.byteLength; i = i + 2) {
      var v = view.getInt16(i, 1)
      var j = d.display_items.length;
      var c = OSD.constants.DISPLAY_FIELDS[j];
      d.display_items.push({
        name: c.name,
        index: j,
        position: v,
        positionable: c.positionable,
        preview: c.preview
      });
    }
    OSD.updateDisplaySize();
  }
};

OSD.GUI = {};
OSD.GUI.preview = {
  onDragStart: function(e) {
    var ev = e.originalEvent;
    ev.dataTransfer.setData("text/plain", ev.target.id);
    ev.dataTransfer.setDragImage($(this).data('field').preview_img, 6, 9);
  },
  onDragOver: function(e) {
    var ev = e.originalEvent;
    ev.preventDefault();
    ev.dataTransfer.dropEffect = "move"
    $(this).css({
      background: 'rgba(0,0,0,.5)'
    });
  },
  onDragLeave: function(e) {
    // brute force unstyling on drag leave
    $(this).removeAttr('style');
  },
  onDrop: function(e) {
    var ev = e.originalEvent;
    var position = $(this).removeAttr('style').data('position');
    var field_id = parseInt(ev.dataTransfer.getData('text').split('field-')[1])
    var display_item = OSD.data.display_items[field_id];
    var overflows_line = FONT.constants.SIZES.LINE - ((position % FONT.constants.SIZES.LINE) + display_item.preview.length);
    if (overflows_line < 0) {
      position += overflows_line;
    }
    if (position > OSD.data.display_size.total/2) {
      position = position - OSD.data.display_size.total;
    }
    $('input.'+field_id+'.position').val(position).change();
  },
};


TABS.osd = {};
TABS.osd.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'osd') {
        GUI.active_tab = 'osd';
    }

    $('#content').load("./tabs/osd.html", function () {
        // translate to user-selected language
        localize();

        // Open modal window
        new jBox('Modal', {
            width: 600,
            height: 240,
            closeButton: 'title',
            animation: false,
            attach: $('#fontmanager'),
            title: 'OSD Font Manager',
            content: $('#fontmanagercontent')
        });

        // 2 way binding... sorta
        function updateOsdView() {
          // ask for the OSD config data
          MSP.promise(MSPCodes.MSP_OSD_CONFIG)
          .then(function(info) {
            if (!info.length) {
              $('.unsupported').fadeIn();
              return;
            }
            $('.supported').fadeIn();
            OSD.msp.decode(info);
            // video mode
            var $videoTypes = $('.video-types').empty();
            for (var i = 0; i < OSD.constants.VIDEO_TYPES.length; i++) {
              var type = OSD.constants.VIDEO_TYPES[i];
              var $checkbox = $('<label/>').append($('<input name="video_system" type="radio"/>'+type+'</label>')
                .prop('checked', i === OSD.data.video_system)
                .data('type', type)
                .data('type', i)
              );
              $videoTypes.append($checkbox);
            }
            $videoTypes.find(':radio').click(function(e) {
              OSD.data.video_system = $(this).data('type');
              MSP.promise(MSPCodes.MSP_SET_OSD_CONFIG, OSD.msp.encodeOther())
              .then(function() {
                updateOsdView();
              });
            });

            // display fields on/off and position
            var $displayFields = $('.display-fields').empty();
            for (let field of OSD.data.display_items) {
              var checked = (-1 != field.position) ? 'checked' : '';
              var $field = $('<div class="display-field"/>');
              $field.append(
                $('<input type="checkbox" name="'+field.name+'" class="togglesmall"></input>')
                .data('field', field)
                .attr('checked', field.position != -1)
                .change(function(e) {
                  var field = $(this).data('field');
                  var $position = $(this).parent().find('.position.'+field.name);
                  if (field.position == -1) {
                    $position.show();
                    field.position = OSD.data.last_positions[field.name]
                  }
                  else {
                    $position.hide();
                    OSD.data.last_positions[field.name] = field.position
                    field.position = -1
                  }
                  MSP.promise(MSPCodes.MSP_SET_OSD_CONFIG, OSD.msp.encode(field))
                  .then(function() {
                    updateOsdView();
                  });
                })
              );
              $field.append('<label for="'+field.name+'">'+inflection.titleize(field.name)+'</label>');
              if (field.positionable && field.position != -1) {
                $field.append(
                  $('<input type="number" class="'+field.index+' position"></input>')
                  .data('field', field)
                  .val(field.position)
                  .change($.debounce(250, function(e) {
                    var field = $(this).data('field');
                    var position = parseInt($(this).val());
                    field.position = position;
                    MSP.promise(MSPCodes.MSP_SET_OSD_CONFIG, OSD.msp.encode(field))
                    .then(function() {
                      updateOsdView();
                    });
                  }))
                );
              }
              $displayFields.append($field);
            }
            GUI.switchery();
            // buffer the preview
            OSD.data.preview = [];
            OSD.data.display_size.total = OSD.data.display_size.x * OSD.data.display_size.y;
            // clear the buffer
            for(var i = 0; i < OSD.data.display_size.total; i++) {
              OSD.data.preview.push([null, ' '.charCodeAt(0)]);
            }
            // draw all the displayed items and the drag and drop preview images
            for(let field of OSD.data.display_items) {
              if (!field.preview || field.position == -1) { continue; }
              var j = (field.position >= 0) ? field.position : field.position + OSD.data.display_size.total;
              // create the preview image
              field.preview_img = new Image();
              var canvas = document.createElement('canvas');
              var ctx = canvas.getContext("2d");
              // fill the screen buffer
              for(var i = 0; i < field.preview.length; i++) {
                var charCode = field.preview.charCodeAt(i);
                OSD.data.preview[j++] = [field, charCode];
                // draw the preview
                var img = new Image();
                img.src = FONT.draw(charCode);
                ctx.drawImage(img, i*12, 0);
              }
              field.preview_img.src = canvas.toDataURL('image/png');
            }
            // logo
            var x = 160;
            for (var i = 1; i < 5; i++) {
              for (var j = 3; j < 27; j++)
                  OSD.data.preview[i * 30 + j] = [{name: 'LOGO', positionable: false}, x++];
            }
            var centerishPosition = 194;
            // artificial horizon
            if ($('input[name="ARTIFICIAL_HORIZON"]').prop('checked')) {
              for (var i = 0; i < 9; i++) {
                OSD.data.preview[centerishPosition - 4 + i] = SYM.AH_BAR9_0 + 4;
              }
              OSD.data.preview[centerishPosition - 1] = SYM.AH_CENTER_LINE;
              OSD.data.preview[centerishPosition + 1] = SYM.AH_CENTER_LINE_RIGHT;
              OSD.data.preview[centerishPosition]     = SYM.AH_CENTER;
            }
            // sidebars
            if ($('input[name="HORIZON_SIDEBARS"]').prop('checked')) {
              var hudwidth  = OSD.constants.AHISIDEBARWIDTHPOSITION;
              var hudheight = OSD.constants.AHISIDEBARHEIGHTPOSITION;
              for (var i = -hudheight; i <= hudheight; i++) {
                OSD.data.preview[centerishPosition - hudwidth + (i * FONT.constants.SIZES.LINE)] = SYM.AH_DECORATION;
                OSD.data.preview[centerishPosition + hudwidth + (i * FONT.constants.SIZES.LINE)] = SYM.AH_DECORATION;
              }
              // AH level indicators
              OSD.data.preview[centerishPosition-hudwidth+1] =  SYM.AH_LEFT;
              OSD.data.preview[centerishPosition+hudwidth-1] =  SYM.AH_RIGHT;
            }
            // render
            var $preview = $('.display-layout .preview').empty();
            var $row = $('<div class="row"/>');
            for(var i = 0; i < OSD.data.display_size.total;) {
              var charCode = OSD.data.preview[i];
              if (typeof charCode === 'object') {
                var field = OSD.data.preview[i][0];
                var charCode = OSD.data.preview[i][1];
              }
              var $img = $('<div class="char"><img src='+FONT.draw(charCode)+'></img></div>')
                .on('dragover', OSD.GUI.preview.onDragOver)
                .on('dragleave', OSD.GUI.preview.onDragLeave)
                .on('drop', OSD.GUI.preview.onDrop)
                .data('position', i);
              if (field && field.positionable) {
                $img
                  .attr('id', 'field-'+field.index)
                  .data('field', field)
                  .prop('draggable', true)
                  .on('dragstart', OSD.GUI.preview.onDragStart);
              }
              else {
              }
              $row.append($img);
              if (++i % OSD.data.display_size.x == 0) {
                $preview.append($row);
                $row = $('<div class="row"/>');
              }
            }
          });
        };

        $('a.save').click(function() {
          var self = this;
          MSP.promise(MSPCodes.MSP_EEPROM_WRITE);
          GUI.log('OSD settings saved');
          var oldText = $(this).text();
          $(this).html("Saved");
          setTimeout(function () {
              $(self).html(oldText);
          }, 2000);
        });

        // font preview window
        var $preview = $('.font-preview');

        //  init structs once, also clears current font
        FONT.initData();

        var $fontPicker = $('.fontbuttons button');
        $fontPicker.click(function(e) {
          if (!$(this).data('font-file')) { return; }
          $fontPicker.removeClass('active');
          $(this).addClass('active');
          $.get('/resources/osd/' + $(this).data('font-file') + '.mcm', function(data) {
            FONT.parseMCMFontFile(data);
            FONT.preview($preview);
            updateOsdView();
          });
        });

        // load the first font when we change tabs
        $fontPicker.first().click();

        $('button.load_font_file').click(function() {
          $fontPicker.removeClass('active');
          FONT.openFontFile().then(function() {
            FONT.preview($preview);
            updateOsdView();
          });
        });

        // font upload
        $('a.flash_font').click(function () {
            if (!GUI.connect_lock) { // button disabled while flashing is in progress
                $('.progressLabel').text('Uploading...');
                FONT.upload($('.progress').val(0)).then(function() {
                    var msg = 'Uploaded all ' + FONT.data.characters.length + ' characters';
                    console.log(msg);
                    $('.progressLabel').text(msg);
                });
            }
        });

        $(document).on('click', 'span.progressLabel a.save_font', function () {
            chrome.fileSystem.chooseEntry({type: 'saveFile', suggestedName: 'baseflight', accepts: [{extensions: ['mcm']}]}, function (fileEntry) {
                if (chrome.runtime.lastError) {
                    console.error(chrome.runtime.lastError.message);
                    return;
                }

                chrome.fileSystem.getDisplayPath(fileEntry, function (path) {
                    console.log('Saving firmware to: ' + path);

                    // check if file is writable
                    chrome.fileSystem.isWritableEntry(fileEntry, function (isWritable) {
                        if (isWritable) {
                            var blob = new Blob([intel_hex], {type: 'text/plain'});

                            fileEntry.createWriter(function (writer) {
                                var truncated = false;

                                writer.onerror = function (e) {
                                    console.error(e);
                                };

                                writer.onwriteend = function() {
                                    if (!truncated) {
                                        // onwriteend will be fired again when truncation is finished
                                        truncated = true;
                                        writer.truncate(blob.size);

                                        return;
                                    }
                                };

                                writer.write(blob);
                            }, function (e) {
                                console.error(e);
                            });
                        } else {
                            console.log('You don\'t have write permissions for this file, sorry.');
                            GUI.log('You don\'t have <span style="color: red">write permissions</span> for this file');
                        }
                    });
                });
            });
        });

        $(document).keypress(function (e) {
            if (e.which == 13) { // enter
                // Trigger regular Flashing sequence
                $('a.flash_font').click();
            }
        });

        GUI.content_ready(callback);
    });
};

TABS.osd.cleanup = function (callback) {
    PortHandler.flush_callbacks();

    // unbind "global" events
    $(document).unbind('keypress');
    $(document).off('click', 'span.progressLabel a');

    if (callback) callback();
};
