/*
---
description: jBox is a powerful and flexible jQuery plugin, taking care of all your modal windows, tooltips, notices and more.

authors: Stephan Wagner (http://stephanwagner.me)

license: MIT (http://opensource.org/licenses/MIT)

requires: jQuery 1.11.1 (http://code.jquery.com/jquery-1.11.1.min.js)
          jQuery 2.1.1 (http://code.jquery.com/jquery-2.1.1.min.js)

documentation: http://stephanwagner.me/jBox/documentation

demos: http://stephanwagner.me/jBox/demos
...
*/

(function (root, factory) {
	
	// AMD. Register as an anonymous module
    if (typeof define === 'function' && define.amd) {
        define(['jQuery'], function (jQuery) {
            return (root.jBox = factory(jQuery));
        });
    
    // Node. Does not work with strict CommonJS, but only CommonJS-like enviroments that support module.exports, like Node
    } else if (typeof exports === 'object') {
        module.exports = factory(require('jQuery'));
    
    // Browser globals
    } else {
        root.jBox = factory(root.jQuery);
    }
}(this, function (jQuery) {
	
	var jBox = function (type, options) {
		
		this.options = {
			
			// jBox ID
			id: null,					// Choose a unique id, otherwise jBox will set one for you (jBoxID1, jBoxID2, ...)
			
			// Dimensions
			width: 'auto',				// Width of content area (e.g. 'auto', 100)
			height: 'auto',				// Height of content area
			minWidth: null,				// Minimum width
			maxHeight: null,			// Minimum height
			minWidth: null,				// Maximum width
			maxHeight: null,			// Minimum height
			
			// Attach
			attach: null,				// Attach jBox to elements (if no target element is provided, jBox will use the attached element as target)
			trigger: 'click',			// The event to open or close your jBoxes, use 'click' or 'mouseenter'
			preventDefault: false,		// Prevent default event when opening jBox (e.g. don't follow the href in a link when clicking on it)
			
			// Content
			title: null,				// Adds a title to your jBox
			content: null,				// You can use a string to set text or HTML as content, or an element selector (e.g. jQuery('#jBox-content')) to append one or several elements (elements appended will get style display: 'block', so hide them with CSS style display: 'none' beforehand)
			getTitle: null,				// Get the title from an attribute when jBox opens
			getContent: null,			// Get the content from an attribute when jBox opens
			isolateScroll: true,		// Isolates scrolling to content container
			
			// AJAX request
			ajax: {						// Setting an url will make an AJAX call when jBox opens
				url: null,				// URL to send the AJAX request to
				data: '',				// Data to send with your AJAX call (e.g. 'id=82&limit=10')
				// Optional you can add any jQuery AJAX option (http://api.jquery.com/jquery.ajax/)
				reload: false,			// Resend the ajax call every time jBox opens
				getData: 'data-ajax',	// The attribute in the source element where the AJAX will look for the data to send with, e.g. data-ajax="id=82&limit=10"
				setContent: true,		// Automatically set the response as new content when the AJAX call is finished
				spinner: true			// Hides the current content and adds a spinner while loading, you can pass html content to add your own spinner, e.g. spinner: '<div class="mySpinner"></div>'
			},
			
			// Position
			target: null,				// The target element where jBox will be opened
			position: {
				x: 'center',			// Horizontal Position (Use a number, 'left', 'right' or 'center')
				y: 'center'				// Vertical Position (Use a number, 'top', 'bottom' or 'center')
			},
			outside: null,				// Use 'x', 'y', or 'xy' to move your jBox outside of the target element
			offset: 0,					// Offset to final position, you can set different values for x and y with an object e.g. {x: 15, y: 0}
			
			attributes: {				// Note that attributes can only be 'left' or 'right' when using numbers for position, e.g. {x: 300, y: 20}
				x: 'left',				// Horizontal position, use 'left' or 'right'
				y: 'top'				// Vertical position, use 'top' or 'bottom'
			},
			adjustPosition: false,		// Adjusts the position when there is not enough space (use true, 'flip' or 'move')
			adjustTracker: false,		// By default jBox adjusts the position when opening, to adjust when scrolling or resizing, use 'scroll', 'resize' or 'true' (both events)
			adjustDistance: 5,			// How far from the window edge we start adjusting, use an object to set different values: {bottom: 5, top: 50, left: 5, right: 20}
			fixed: false,				// Your jBox will stay on position when scrolling
			reposition: false,			// Calculates new position when the window-size changes
			repositionOnOpen: true,		// Calculates new position each time jBox opens (rather than only when it opens the first time)
			repositionOnContent: true,	// Calculates new position when the content changes with .setContent() or .setTitle()
			
			// Pointer
			pointer: false,				// Your pointer will always point towards the target element, so the option outside should be 'x' or 'y'
			pointTo: 'target',			// Setting something else than 'target' will add a pointer even if there is no target element set or found (Use 'top', 'bottom', 'left' or 'right')
			
			// Animations
			fade: 180,					// Fade duration in ms, set to 0 or false to disable
			animation: null,			// Animation when opening or closing (use 'pulse', 'zoomIn', 'zoomOut', 'move', 'slide', 'flip', 'tada') (CSS inspired from Daniel Edens Animate.css: http://daneden.me/animate)
			
			// Appearance
			theme: 'Default',			// Set a jBox theme class
			addClass: '',				// Adds classes to the wrapper
			overlay: false,				// Adds an overlay when jBox opens (set color and opacity with CSS)
			zIndex: 10000,				// Use a high zIndex (your overlay will have the lowest zIndex of all your jBoxes (with overlays) minus one)
			
			// Delays
			delayOpen: 0,				// Delay opening in ms (Note that the delay will be ignored if your jBox didn't finish closing)
			delayClose: 0,				// Delay closing in ms (Note that there is always a closing delay of at least 10ms to ensure jBox won't be closed when opening right away)
			
			// Closing events
			closeOnEsc: false,			// Close jBox when pressing [esc] key
			closeOnClick: false,		// Close jBox with mouseclick, use 'true' (click anywhere), 'box' (click on jBox itself), 'overlay' (click on the overlay), 'body' (click anywhere but jBox)
			closeOnMouseleave: false,	// Close jBox when the mouse leaves the jBox area or the area of the attached element
			closeButton: false,			// Adds a close button to your jBox, use 'title', 'overlay', 'box' or true (true will add the button to overlay, title or box, in that order if any of those elements can be found)
			
			// Other options
			constructOnInit: false,		// Construct jBox when it's being initialized
			blockScroll: false,			// When jBox is open, block scrolling
			appendTo: jQuery('body'), 	// Provide an element if you want the jBox to be positioned inside a specific element (only useful for fixed positions or when position values are numbers)
			draggable: null,			// Make your jBox draggable (use 'true', 'title' or provide an element as handle) (inspired from Chris Coyiers CSS-Tricks http://css-tricks.com/snippets/jquery/draggable-without-jquery-ui/)
			dragOver: true,				// When you have multiple draggable jBoxes, the one you select will always move over the other ones
			
			// Events					// Note: You can use 'this' in the event functions, it refers to your jBox object (e.g. onInit: function() { this.open(); })
			onInit: null,				// Triggered when jBox is initialized
			onBeforeInit: null,			// Triggered when jBox starts initializing, useful to add your own internal functions
			onAttach: null,				// Triggered when jBox attached itself to elements
			// TODO onPosition
			onCreated: null,			// Triggered when jBox is created and is availible in DOM
			onOpen: null,				// Triggered when jBox opens
			onClose: null,				// Triggered when jBox closes
			onCloseComplete: null,		// Triggered when jBox is completely closed (when fading is finished, useful if you want to destroy the jBox when it is closed)
			
			// Only for type "Confirm"
			confirmButton: 'Submit',	// Text for the submit button
			cancelButton: 'Cancel',		// Text for the cancel button
			confirm: null,				// Function to execute when clicking the submit button. By default jBox will use firstly the onclick and secondly the href attribute
			cancel: null,				// Function to execute when clicking the cancel button
			
			// Only for type "Notice"
			autoClose: 7000,			// Time when jBox should close automatically
			color: null,				// Makes your notices colorful, use 'black', 'red', 'green', 'blue', 'yellow'
			stack: true,				// Set to false to disable notice-stacking
			audio: false,				// Set the url to an audio file without extention, e.g. '/url/filename'. jBox will look for an .mp3 and an .ogg file
			volume: 100,				// Percent of volume for audio files
			
			// Only for type "Image"
			src: 'href',				// The attribute where jBox gets the image source from, e.g. href="/path_to_image/image.jpg"
			gallery: 'data-jbox-image',	// The attribute where you define the image gallery, e.g. data-jbox-image="gallery1"
			imageLabel: 'title',		// The attribute where jBox gets the image label from, e.g. title="My label"
			imageFade: 600,				// The fade duration for images
			imageSize: 'contain'		// How to display the images: Use CSS background-position values, e.g. 'cover', 'contain', 'auto', 'initial', '50% 50%'
		};

		// Default type options
		this.defaultOptions = {
			// Default options for tooltips
			'Tooltip': {
				getContent: 'title',
				trigger: 'mouseenter',
				position: {x: 'center', y: 'top'},
				outside: 'y',
				pointer: true,
				adjustPosition: true,
				reposition: true
			},
			// Default options for mouse tooltips
			'Mouse': {
				target: 'mouse',
				position: {x: 'right', y: 'bottom'},
				offset: 15,
				trigger: 'mouseenter',
				adjustPosition: 'flip'
			},
			// Default options for modal windows
			'Modal': {
				target: jQuery(window),
				fixed: true,
				blockScroll: true,
				closeOnEsc: true,
				closeOnClick: 'overlay',
				closeButton: true,
				overlay: true,
				animation: 'zoomOut'
			},
			// Default options for modal confirm windows
			'Confirm': {
				target: jQuery(window),
				fixed: true,
				attach: jQuery('[data-confirm]'),
				getContent: 'data-confirm',
				content: 'Do you really want to do this?',
				minWidth: 320,
				maxWidth: 460,
				blockScroll: true,
				closeOnEsc: true,
				closeOnClick: 'overlay',
				closeButton: true,
				overlay: true,
				animation: 'zoomOut',
				preventDefault: true,
				_onAttach: function(el) {
					// Extract the href or the onclick event if no submit event is passed
					if (!this.options.confirm) {
						var submit = el.attr('onclick') ? el.attr('onclick') : (el.attr('href') ? (el.attr('target') ? 'window.open("' + el.attr('href') + '", "' + el.attr('target') + '");'  : 'window.location.href = "' + el.attr('href') + '";') : '');
						el.prop('onclick', null).data('jBox-Confirm-submit', submit);
					}
				},
				_onCreated: function() {
					// Add a footer to the jBox container
					this.footer = jQuery('<div class="jBox-Confirm-footer"/>');
					jQuery('<div class="jBox-Confirm-button jBox-Confirm-button-cancel"/>').html(this.options.cancelButton).click(function() { this.options.cancel && this.options.cancel(); this.close(); }.bind(this)).appendTo(this.footer);
					this.submitButton = jQuery('<div class="jBox-Confirm-button jBox-Confirm-button-submit"/>').html(this.options.confirmButton).appendTo(this.footer);
					this.footer.appendTo(this.container);
				},
				_onOpen: function() {
					// Set the new action for the submit button
					this.submitButton.off('click.jBox-Confirm' + this.id).on('click.jBox-Confirm' + this.id, function() { this.options.confirm ? this.options.confirm() : eval(this.source.data('jBox-Confirm-submit')); this.close(); }.bind(this));
				}
			},
			// Default options for notices
			'Notice': {
				target: jQuery(window),
				fixed: true,
				position: {x: 20, y: 20},
				attributes: {x: 'right', y: 'top'},
				animation: 'zoomIn',
				closeOnClick: 'box',
				_onInit: function () {
					this.open();
					this.options.delayClose = this.options.autoClose;
					this.options.delayClose && this.close();
				},
				_onCreated: function() {
					this.options.color && this.wrapper.addClass('jBox-Notice-color jBox-Notice-' + this.options.color);
					this.wrapper.data('jBox-Notice-position', this.options.attributes.x + '-' + this.options.attributes.y);
				},
				_onOpen: function() {
					// Loop through notices at same window corner and either move or destroy them
					jQuery.each(jQuery('.jBox-Notice'), function(index, el) {
						el = jQuery(el);
						
						if (el.attr('id') == this.id || el.data('jBox-Notice-position') != this.options.attributes.x + '-' + this.options.attributes.y) return;
						if (!this.options.stack) {
							el.data('jBox').close({ignoreDelay: true});
							return;
						}
						el.css('margin-' + this.options.attributes.y, parseInt(el.css('margin-' + this.options.attributes.y)) + this.wrapper.outerHeight() + 10);
					}.bind(this));
					
					// Play audio file, IE8 doesn't support audio
					this.options.audio && this.audio({url: this.options.audio, valume: this.options.volume});
				},
				// Remove notice from DOM when closing finishes
				_onCloseComplete: function() {
					this.destroy();
				}
			},
			// Default options for images
			'Image': {
				target: jQuery(window),
				fixed: true,
				blockScroll: true,
				closeOnEsc: true,
				closeOnClick: 'overlay',
				closeButton: true,
				overlay: true,
				animation: 'zoomOut',
				width: 800,
				height: 533,
				attach: jQuery('[data-jbox-image]'),
				preventDefault: true,
				
				// TODO: What if the image is not found?
				// TODO: What if the first image of a gallery needs some time to load, but other images are in content. Maybe add a blank black container
				
				_onInit: function() {
					this.images = this.currentImage = {};
					this.imageZIndex = 1;
					
					// Loop through images, sort and save in global variable
					this.attachedElements && jQuery.each(this.attachedElements, function (index, item) {
						item = jQuery(item);
						if (item.data('jBox-image-gallery')) return;
						var gallery = item.attr(this.options.gallery) || 'default';
						!this.images[gallery] && (this.images[gallery] = []);
						this.images[gallery].push({src: item.attr(this.options.src), label: (item.attr(this.options.imageLabel) || '')});
						
						// Remove the title attribute so it won't show the browsers tooltip
						this.options.imageLabel == 'title' && item.removeAttr('title');
						
						// Store data in source element for easy access
						item.data('jBox-image-gallery', gallery);
						item.data('jBox-image-id', (this.images[gallery].length - 1));
					}.bind(this));
					
					// Helper to inject the image into content area
					var appendImage = function(gallery, id, preload, open) {
						if (jQuery('#jBox-image-' + gallery + '-' + id).length) return;
						
						var image = jQuery('<div/>', {
							id: 'jBox-image-' + gallery + '-' + id,
							'class': 'jBox-image-container'
						}).css({
							backgroundImage: 'url(' + this.images[gallery][id].src + ')',
							backgroundSize: this.options.imageSize,
							opacity: (open ? 1 : 0),
							zIndex: (preload ? 0 : this.imageZIndex++)
						}).appendTo(this.content);
						
						var text = jQuery('<div/>', {
							id: 'jBox-image-label-' + gallery + '-' + id,
							'class': 'jBox-image-label' + (open ? ' active' : '')
						}).html(this.images[gallery][id].label).appendTo(this.imageLabel);
						
						!open && !preload && image.animate({opacity: 1}, this.options.imageFade);
					}.bind(this);
					
					// Helper to show new image label
					var showLabel = function(gallery, id) {
						jQuery('.jBox-image-label.active').removeClass('active');
						jQuery('#jBox-image-label-' + gallery + '-' + id).addClass('active');
					};
					
					// Show images when they are loaded or load them if not
					this.showImage = function(img) {
						// Get the gallery and the image id from the next or the previous image
						if (img != 'open') {
							var gallery = this.currentImage.gallery;
							var id = this.currentImage.id + (1 * (img == 'prev') ? -1 : 1);
							id = id > (this.images[gallery].length - 1) ? 0 : (id < 0 ? (this.images[gallery].length - 1) : id);
							// Or get image data from source element
						} else {
							var gallery = this.source.data('jBox-image-gallery');
							var id = this.source.data('jBox-image-id');
							
							// Remove or show the next and prev buttons
							jQuery('.jBox-image-pointer-prev, .jBox-image-pointer-next').css({display: (this.images[gallery].length > 1 ? 'block' : 'none')});
						}
						
						// Set new current image
						this.currentImage = {gallery: gallery, id: id};
						
						// Show image if it already exists
						if (jQuery('#jBox-image-' + gallery + '-' + id).length) {
							jQuery('#jBox-image-' + gallery + '-' + id).css({zIndex: this.imageZIndex++, opacity: 0}).animate({opacity: 1}, (img == 'open') ? 0 : this.options.imageFade);
							showLabel(gallery, id);
							
							// Load image if not found
						} else {
							// TODO loading not working properly anymore
							this.wrapper.addClass('jBox-loading');
							var image = jQuery('<img src="' + this.images[gallery][id].src + '">').load(function() {
								appendImage(gallery, id, false);
								showLabel(gallery, id);
								this.wrapper.removeClass('jBox-loading');
							}.bind(this));
						}
						
						// Preload next image
						var next_id = id + 1;
						next_id = next_id > (this.images[gallery].length - 1) ? 0 : (next_id < 0 ? (this.images[gallery].length - 1) : next_id);
						
						(!jQuery('#jBox-image-' + gallery + '-' + next_id).length) && jQuery('<img src="' + this.images[gallery][next_id].src + '">').load(function() {
							appendImage(gallery, next_id, true);
						});
					};
				},
				_onCreated: function() {
				
					// TODO: NO ID!!!
				
					this.imageLabel = jQuery('<div/>', {'id': 'jBox-image-label'}).appendTo(this.wrapper);
					this.wrapper.append(jQuery('<div/>', {'class': 'jBox-image-pointer-prev', click: function() { this.showImage('prev'); }.bind(this)})).append(jQuery('<div/>', {'class': 'jBox-image-pointer-next', click: function() { this.showImage('next'); }.bind(this)}));
				},
				_onOpen: function() {
					// Add a class to body so you can control the appearance of the overlay, for images a darker one is better
					jQuery('body').addClass('jBox-image-open');
					
					// Add key events
					jQuery(document).on('keyup.jBox-' + this.id, function(ev) {
						(ev.keyCode == 37) && this.showImage('prev');
						(ev.keyCode == 39) && this.showImage('next');
					}.bind(this));
					
					// Load the image from the attached element
					this.showImage('open');
				},
				_onClose: function() {
					jQuery('body').removeClass('jBox-image-open');
					
					// Remove key events
					jQuery(document).off('keyup.jBox-' + this.id);
				},
				_onCloseComplete: function() {
					// Hide all images
					this.wrapper.find('.jBox-image-container').css('opacity', 0);
				}
			}
		};
		
		// Set default options for jBox types
		if (jQuery.type(type) == 'string') {
			this.type = type;
			type = this.defaultOptions[type] ? this.defaultOptions[type] : window['jBox' + type + 'Options'];
		}
		
		// Merge options
		this.options = jQuery.extend(true, this.options, type, options);
		
		// Function to fire events
		this._fireEvent = function(event, pass) {
			this.options['_' + event] && (this.options['_' + event].bind(this))(pass);
			this.options[event] && (this.options[event].bind(this))(pass);
		};
		
		// Fire onBeforeInit event
		this._fireEvent('onBeforeInit');
		
		// Get unique ID
		if (this.options.id === null) {
			this.options.id = 'jBoxID' + jBox._getUniqueID();
		}
		this.id = this.options.id;
		
		// Correct impossible options
		((this.options.position.x == 'center' && this.options.outside == 'x') || (this.options.position.y == 'center' && this.options.outside == 'y')) && (this.options.outside = false);
		(!this.options.outside || this.options.outside == 'xy') && (this.options.pointer = false);
		
		// Correct multiple choice options
		jQuery.type(this.options.offset) != 'object' && (this.options.offset = {x: this.options.offset, y: this.options.offset});
		this.options.offset.x || (this.options.offset.x = 0);
		this.options.offset.y || (this.options.offset.y = 0);
		jQuery.type(this.options.adjustDistance) != 'object' ? (this.options.adjustDistance = {top: this.options.adjustDistance, right: this.options.adjustDistance, bottom: this.options.adjustDistance, left: this.options.adjustDistance}) : (this.options.adjustDistance = jQuery.extend({top: 5, left: 5, right: 5, bottom: 5}, this.options.adjustDistance));
		
		// Save where the jBox is aligned to
		this.align = (this.options.outside && this.options.outside != 'xy') ? this.options.position[this.options.outside] : (this.options.position.y != 'center' && jQuery.type(this.options.position.y) != 'number' ? this.options.position.x : (this.options.position.x != 'center' && jQuery.type(this.options.position.x) != 'number' ? this.options.position.y : this.options.attributes.x));
		
		// Save default outside position
		this.options.outside && this.options.outside != 'xy' && (this.outside = this.options.position[this.options.outside]);
		
		// I know browser detection is bad practice, but for now it seems the only option to get jBox working in IE8
		var userAgent = navigator.userAgent.toLowerCase();
		this.IE8 = userAgent.indexOf('msie') != -1 && parseInt(userAgent.split('msie')[1]) == 8;
		
		// Save global var for webkit prefix
		this.prefix = userAgent.indexOf('webkit') != -1 ? '-webkit-' : '';
		
		// Internal functions, used to easily get values
		this._getOpp = function(opp) { return {left: 'right', right: 'left', top: 'bottom', bottom: 'top', x: 'y', y: 'x'}[opp]; };
		this._getXY = function(xy) { return {left: 'x', right: 'x', top: 'y', bottom: 'y', center: 'x'}[xy]; };
		this._getTL = function(tl) { return {left: 'left', right: 'left', top: 'top', bottom: 'top', center: 'left', x: 'left', y: 'top'}[tl]; };
		
		// Check for SVG support
		this._supportsSVG = function() {
			return document.createElement('svg').getAttributeNS;
		}
		
		// Create an svg element
		this._createSVG = function(type, options) {
			var svg = document.createElementNS('http://www.w3.org/2000/svg', type);
			jQuery.each(options, function (index, item) {
				svg.setAttribute(item[0], (item[1] || ''));
			});
			return svg;
		};
		
		// Append a svg element to a svg container
		this._appendSVG = function(source, target) {
			return target.appendChild(source);
		};
		
		// Isolate scrolling in a container
		this._isolateScroll = function(el) {
			if(!el || !jQuery(el).length) return;
			el.on('DOMMouseScroll.jBoxIsolatedScroll mousewheel.jBoxIsolatedScroll', function(ev) {
				var delta = ev.wheelDelta || (ev.originalEvent && ev.originalEvent.wheelDelta) || -ev.detail,
					overflowBottom = this.scrollTop + el.outerHeight() - this.scrollHeight >= 0,
					overflowTop = this.scrollTop <= 0;
					if ((delta < 0 && overflowBottom) || (delta > 0 && overflowTop)) ev.preventDefault();
			});
		};
		
		// Create jBox
		this._create = function() {
			if (this.wrapper) return;
			
			// Create wrapper
			this.wrapper = jQuery('<div/>', {
				id: this.id,
				'class': 'jBox-wrapper' + (this.type ? ' jBox-' + this.type : '') + (this.options.theme ? ' jBox-' + this.options.theme : '') + (this.options.addClass ? ' ' + this.options.addClass : '') + (this.IE8 ? ' jBox-IE8' : '')
			}).css({
				position: (this.options.fixed ? 'fixed' : 'absolute'),
				display: 'none',
				opacity: 0,
				zIndex: this.options.zIndex
				
				// Save the jBox instance in the wrapper, so you can get access to your jBox when you only have the element
			}).data('jBox', this);
			
			// Add mouseleave event (.parents('*') might be a performance nightmare! Maybe there is a better way)
			this.options.closeOnMouseleave && this.wrapper.mouseleave(function(ev) {
				// Only close when the new target is not the source element
				!this.source || !(ev.relatedTarget == this.source[0] || jQuery.inArray(this.source[0], jQuery(ev.relatedTarget).parents('*')) !== -1) && this.close();
			}.bind(this));
			
			// Add closeOnClick: 'box' events
			(this.options.closeOnClick == 'box') && this.wrapper.on('touchend click', function() { this.close({ignoreDelay: true}); }.bind(this));
			
			// Create container
			this.container = jQuery('<div/>', {'class': 'jBox-container'}).appendTo(this.wrapper);
			
			// Create content
			this.content = jQuery('<div/>', {'class': 'jBox-content'}).css({width: this.options.width, height: this.options.height, minWidth: this.options.minWidth, minHeight: this.options.minHeight, maxWidth: this.options.maxWidth, maxHeight: this.options.maxHeight}).appendTo(this.container);
			
			// Isolate scrolling
			this.options.isolateScroll && this._isolateScroll(this.content);
			
			// Create close button
			if (this.options.closeButton) {
				this.closeButton = jQuery('<div/>', {'class': 'jBox-closeButton jBox-noDrag'}).on('touchend click', function(ev) { this.isOpen && this.close({ignoreDelay: true}); }.bind(this));
				
				if (this._supportsSVG()) {
					var closeButtonSVG = this._createSVG('svg', [['viewBox', '0 0 24 24']]);
					this._appendSVG(this._createSVG('path', [['d', 'M22.2,4c0,0,0.5,0.6,0,1.1l-6.8,6.8l6.9,6.9c0.5,0.5,0,1.1,0,1.1L20,22.3c0,0-0.6,0.5-1.1,0L12,15.4l-6.9,6.9c-0.5,0.5-1.1,0-1.1,0L1.7,20c0,0-0.5-0.6,0-1.1L8.6,12L1.7,5.1C1.2,4.6,1.7,4,1.7,4L4,1.7c0,0,0.6-0.5,1.1,0L12,8.5l6.8-6.8c0.5-0.5,1.1,0,1.1,0L22.2,4z']]), closeButtonSVG);
					this.closeButton.append(closeButtonSVG);
				} else {
					this.wrapper.addClass('jBox-nosvg');
				}
				
				// Add close button to jBox container
				if (this.options.closeButton == 'box' || (this.options.closeButton === true && !this.options.overlay && !this.options.title)) {
					this.wrapper.addClass('jBox-closeButton-box');
					this.closeButton.appendTo(this.container);
				}
			}
			
			// Append jBox to DOM
			this.wrapper.appendTo(this.options.appendTo);
			
			// Create pointer
			if (this.options.pointer) {
				
				// Get pointer vars and save globally
				this.pointer = {
					position: (this.options.pointTo != 'target') ? this.options.pointTo : this._getOpp(this.outside),
					xy: (this.options.pointTo != 'target') ? this._getXY(this.options.pointTo) : this._getXY(this.outside),
					align: 'center',
					offset: 0
				};
				
				this.pointer.element = jQuery('<div/>', {'class': 'jBox-pointer jBox-pointer-' + this.pointer.position}).appendTo(this.wrapper);
				this.pointer.dimensions = {
					x: this.pointer.element.outerWidth(),
					y: this.pointer.element.outerHeight()
				};
				
				if (jQuery.type(this.options.pointer) == 'string') {
					var split = this.options.pointer.split(':');
					split[0] && (this.pointer.align = split[0]);
					split[1] && (this.pointer.offset = parseInt(split[1]));
				}
				this.pointer.alignAttribute = (this.pointer.xy == 'x' ? (this.pointer.align == 'bottom' ? 'bottom' : 'top') : (this.pointer.align == 'right' ? 'right' : 'left'));
				
				// Set wrapper CSS
				this.wrapper.css('padding-' + this.pointer.position, this.pointer.dimensions[this.pointer.xy]);
				
				// Set pointer CSS
				this.pointer.element.css(this.pointer.alignAttribute, (this.pointer.align == 'center' ? '50%' : 0)).css('margin-' + this.pointer.alignAttribute, this.pointer.offset);
				this.pointer.margin = {}; this.pointer.margin['margin-' + this.pointer.alignAttribute] = this.pointer.offset;
				
				// Add a transform to fix centered position
				(this.pointer.align == 'center') && this.pointer.element.css(this.prefix + 'transform', 'translate(' + (this.pointer.xy == 'y' ? (this.pointer.dimensions.x * -0.5 + 'px') : 0) + ', ' + (this.pointer.xy == 'x' ? (this.pointer.dimensions.y * -0.5 + 'px') : 0) + ')');
				
				this.pointer.element.css((this.pointer.xy == 'x' ? 'width' : 'height'), parseInt(this.pointer.dimensions[this.pointer.xy]) + parseInt(this.container.css('border-' + this.pointer.alignAttribute + '-width')));
				
				// Add class to wrapper for CSS access
				this.wrapper.addClass('jBox-pointerPosition-' + this.pointer.position);
			}
			
			// Set title and content
			this.setContent(this.options.content, true);
			this.setTitle(this.options.title, true);
			
			// Make jBox draggable
			if (this.options.draggable) {
				var handle = (this.options.draggable == 'title') ? this.titleContainer : (this.options.draggable.length > 0 ? this.options.draggable : (this.options.draggable.selector ? jQuery(this.options.draggable.selector) : this.wrapper));
				handle.addClass('jBox-draggable').on('mousedown', function(ev) {
					if (ev.button == 2 || jQuery(ev.target).hasClass('jBox-noDrag') || jQuery(ev.target).parents('.jBox-noDrag').length) return;
					
					if (this.options.dragOver && this.wrapper.css('zIndex') <= jBox.zIndexMax) {
						jBox.zIndexMax += 1;
						this.wrapper.css('zIndex', jBox.zIndexMax);
					}
					
					var drg_h = this.wrapper.outerHeight(),
						drg_w = this.wrapper.outerWidth(),
						pos_y = this.wrapper.offset().top + drg_h - ev.pageY,
						pos_x = this.wrapper.offset().left + drg_w - ev.pageX;
					jQuery(document).on('mousemove.jBox-draggable-' + this.id, function(ev) {
						this.wrapper.offset({
							top: ev.pageY + pos_y - drg_h,
							left: ev.pageX + pos_x - drg_w
						});
					}.bind(this));
					ev.preventDefault();
				}.bind(this)).on('mouseup', function() { jQuery(document).off('mousemove.jBox-draggable-' + this.id); }.bind(this));
				
				// Add z-index
				jBox.zIndexMax = !jBox.zIndexMax ? this.options.zIndex : Math.max(jBox.zIndexMax, this.options.zIndex);
			}
			
			// Fire onCreated event
			this._fireEvent('onCreated');
		};
		
		// Create jBox onInit
		this.options.constructOnInit && this._create();
		
		// Attach jBox
		this.options.attach && this.attach();
		
		// Position jBox on mouse
		this._positionMouse = function(ev) {
			
			// Calculate positions
			this.pos = {
				left: ev.pageX,
				top: ev.pageY
			};
			var setPosition = function(a, p) {
				// Set centered position
				if (this.options.position[p] == 'center') {
					this.pos[a] -= Math.ceil(this.dimensions[p] / 2);
					return;
				}
				// Move to left or top
				this.pos[a] += (a == this.options.position[p]) ? ((this.dimensions[p] * -1) - this.options.offset[p]) : this.options.offset[p];
				
				return this.pos[a];
			}.bind(this);
			
			// Set position to wrapper
			this.wrapper.css({
				left: setPosition('left', 'x'),
				top: setPosition('top', 'y')
			});
			
			// Adjust mouse position
			this.targetDimensions = {x: 0, y: 0, left: ev.pageX, top: ev.pageY};
			this._adjustPosition();
		};
		
		// Attach document and window events
		this._attachEvents = function() {
			
			// Closing event: closeOnEsc
			this.options.closeOnEsc && jQuery(document).on('keyup.jBox-' + this.id, function(ev) { if (ev.keyCode == 27) { this.close({ignoreDelay: true}); }}.bind(this));
			
			// Closing event: closeOnClick
			if (this.options.closeOnClick === true || this.options.closeOnClick == 'body') {
				jQuery(document).on('touchend.jBox-' + this.id + ' click.jBox-' + this.id, function(ev) {
					if (this.blockBodyClick || (this.options.closeOnClick == 'body' && (ev.target == this.wrapper[0] || this.wrapper.has(ev.target).length)))
						return;
					this.close({ignoreDelay: true});
				}.bind(this));
			}
			
			// Positioning events
			if (((this.options.adjustPosition && this.options.adjustTracker) || this.options.reposition) && !this.fixed && this.outside) {
				
				var scrollTimer,
					scrollTimerTriggered = 0,
					scrollTriggerDelay = 150;	// Trigger scroll and resize events every 150 ms (set a higher value to improve performance)
					
				// Function to delay positioning event
				var positionDelay = function () {
					var now = new Date().getTime();
					if (!scrollTimer) {
						if (now - scrollTimerTriggered > scrollTriggerDelay) {
							this.options.reposition && this.position();
							this.options.adjustTracker && this._adjustPosition();
							scrollTimerTriggered = now;
						}
						scrollTimer = setTimeout(function() {
							scrollTimer = null;
							scrollTimerTriggered = new Date().getTime();
							this.options.reposition && this.position();
							this.options.adjustTracker && this._adjustPosition();
						}.bind(this), scrollTriggerDelay);
					}
				}.bind(this);
				
				// Trigger position events when scrolling
				(this.options.adjustTracker && this.options.adjustTracker != 'resize') && jQuery(window).on('scroll.jBox-' + this.id, function(ev) { positionDelay(); }.bind(this));
				
				// Trigger position events when resizing
				((this.options.adjustTracker && this.options.adjustTracker != 'scroll') || this.options.reposition) && jQuery(window).on('resize.jBox-' + this.id, function(ev) { positionDelay(); }.bind(this));
			}
			
			// Mousemove events
			this.options.target == 'mouse' && jQuery('body').on('mousemove.jBox-' + this.id, function(ev) { this._positionMouse(ev); }.bind(this));
		};
		
		// Detach document and window events
		this._detachEvents = function() {
			
			// Closing event: closeOnEsc
			this.options.closeOnEsc && jQuery(document).off('keyup.jBox-' + this.id);
			
			// Closing event: closeOnClick
			(this.options.closeOnClick === true || this.options.closeOnClick == 'body') && jQuery(document).off('touchend.jBox-' + this.id + ' click.jBox-' + this.id);
			
			// Positioning events
			if ((this.options.adjustPosition && this.options.adjustTracker) || this.options.reposition) {
				jQuery(window).off('scroll.jBox-' + this.id);
				jQuery(window).off('resize.jBox-' + this.id);
			}
			
			// Mousemove events
			this.options.target == 'mouse' && jQuery('body').off('mousemove.jBox-' + this.id);
		};

		// Add overlay
		this._addOverlay = function() {
			
			// If the overlay isn't cached, set overlay or create it
			if (!this.overlay) {
				// Get the overlay and adjust z-Index
				this.overlay = jQuery('#jBox-overlay').length ? jQuery('#jBox-overlay').css({zIndex: Math.min(jQuery('#jBox-overlay').css('z-index'), (this.options.zIndex - 1))}) : (jQuery('<div/>', {id: 'jBox-overlay'}).css({display: 'none', opacity: 0, zIndex: (this.options.zIndex - 1)}).appendTo(jQuery('body')));
				
				// Add close button to overlay
				(this.options.closeButton == 'overlay' || this.options.closeButton === true) && ((jQuery('#jBox-overlay .jBox-closeButton').length > 0) ? jQuery('#jBox-overlay .jBox-closeButton').on('touchend click', function() { this.isOpen && this.close({ignoreDelay: true}); }.bind(this)) : this.overlay.append(this.closeButton));
				
				// Add closeOnClick: 'overlay' events
				(this.options.closeOnClick == 'overlay') && this.overlay.on('touchend click', function() { this.isOpen && this.close({ignoreDelay: true}); }.bind(this));
			}
			
			// Add jBox to overlay data
			var overlay_data = this.overlay.data('jBox') || {};
			overlay_data['jBox-' + this.id] = true;
			this.overlay.data('jBox', overlay_data);
			
			// Abort if overlay is shown already
			if (this.overlay.css('display') == 'block') return;
			
			// Show overlay
			this.options.fade ? (this.overlay.stop() && this.overlay.animate({opacity: 1}, {
				queue: false,
				duration: this.options.fade,
				start: function() { this.overlay.css({display: 'block'}); }.bind(this)
			})) : this.overlay.css({display: 'block', opacity: 1});
		};
		
		// Remove overlay
		this._removeOverlay = function() {
			
			// Abort if no overlay found
			if (!this.overlay) return;
			
			// Remove jBox from data
			var overlay_data = this.overlay.data('jBox');
			delete overlay_data['jBox-' + this.id];
			this.overlay.data('jBox', overlay_data);
			
			// Hide overlay if no other jBox needs it
			if (jQuery.isEmptyObject(overlay_data)) {
				this.options.fade ? (this.overlay.stop() && this.overlay.animate({opacity: 0}, {
					queue: false,
					duration: this.options.fade,
					complete: function() { this.overlay.css({display: 'none'}); }.bind(this)
				})) : this.overlay.css({display: 'none', opacity: 0});
			}
		};
		
		// Generate CSS for animations and append to header
		this._generateCSS = function() {
			if (this.IE8) return;
			
			// Get open and close animations if none provided
			(jQuery.type(this.options.animation) != 'object') && (this.options.animation = {
				pulse: {open: 'pulse', close: 'zoomOut'},
				zoomIn: {open: 'zoomIn', close: 'zoomIn'},
				zoomOut: {open: 'zoomOut', close: 'zoomOut'},
				move: {open: 'move', close: 'move'},
				slide: {open: 'slide', close: 'slide'},
				flip: {open: 'flip', close: 'flip'},
				tada: {open: 'tada', close: 'zoomOut'}
			}[this.options.animation]);
			
			// Get direction var
			this.options.animation.open && (this.options.animation.open = this.options.animation.open.split(':'));
			this.options.animation.close && (this.options.animation.close = this.options.animation.close.split(':'));
			this.options.animation.openDirection = this.options.animation.open ? this.options.animation.open[1] : null;
			this.options.animation.closeDirection = this.options.animation.close ? this.options.animation.close[1] : null;
			this.options.animation.open && (this.options.animation.open = this.options.animation.open[0]);
			this.options.animation.close && (this.options.animation.close = this.options.animation.close[0]);
			
			// Add 'Open' and 'Close' to animation names
			this.options.animation.open && (this.options.animation.open += 'Open');
			this.options.animation.close && (this.options.animation.close += 'Close');
			
			// All animations
			var animations = {
				pulse: {
					duration: 350,
					css: [['0%', 'scale(1)'], ['50%', 'scale(1.1)'], ['100%', 'scale(1)']]
				},
				zoomInOpen: {
					duration: (this.options.fade || 180),
					css: [['0%', 'scale(0.9)'], ['100%', 'scale(1)']]
				},
				zoomInClose: {
					duration: (this.options.fade || 180),
					css: [['0%', 'scale(1)'], ['100%', 'scale(0.9)']]
				},
				zoomOutOpen: {
					duration: (this.options.fade || 180),
					css: [['0%', 'scale(1.1)'], ['100%', 'scale(1)']]
				},
				zoomOutClose: {
					duration: (this.options.fade || 180),
					css: [['0%', 'scale(1)'], ['100%', 'scale(1.1)']]
				},
				moveOpen: {
					duration: (this.options.fade || 180),
					positions: {top: {'0%': -12}, right: {'0%': 12}, bottom: {'0%': 12}, left: {'0%': -12}},
					css: [['0%', 'translate%XY(%Vpx)'], ['100%', 'translate%XY(0px)']]
				},
				moveClose: {
					duration: (this.options.fade || 180),
					timing: 'ease-in',
					positions: {top: {'100%': -12}, right: {'100%': 12}, bottom: {'100%': 12}, left: {'100%': -12}},
					css: [['0%', 'translate%XY(0px)'], ['100%', 'translate%XY(%Vpx)']]
				},
				slideOpen: {
					duration: 400,
					positions: {top: {'0%': -400}, right: {'0%': 400}, bottom: {'0%': 400}, left: {'0%': -400}},
					css: [['0%', 'translate%XY(%Vpx)'], ['100%', 'translate%XY(0px)']]
				},
				slideClose: {
					duration: 400,
					timing: 'ease-in',
					positions: {top: {'100%': -400}, right: {'100%': 400}, bottom: {'100%': 400}, left: {'100%': -400}},
					css: [['0%', 'translate%XY(0px)'], ['100%', 'translate%XY(%Vpx)']]
				},
				flipOpen: {
					duration: 600,
					css: [['0%', 'perspective(400px) rotateX(90deg)'], ['40%', 'perspective(400px) rotateX(-15deg)'], ['70%', 'perspective(400px) rotateX(15deg)'], ['100%', 'perspective(400px) rotateX(0deg)']]
				},
				flipClose: {
					duration: (this.options.fade || 300),
					css: [['0%', 'perspective(400px) rotateX(0deg)'], ['100%', 'perspective(400px) rotateX(90deg)']]
				},
				tada: {
					duration: 800,
					css: [['0%', 'scale(1)'], ['10%, 20%', 'scale(0.9) rotate(-3deg)'], ['30%, 50%, 70%, 90%', 'scale(1.1) rotate(3deg)'], ['40%, 60%, 80%', 'scale(1.1) rotate(-3deg)'], ['100%', 'scale(1) rotate(0)']]
				}
			};
			
			// Set Open and Close names for standalone animations
			jQuery.each(['pulse', 'tada'], function(index, item) { animations[item + 'Open'] = animations[item + 'Close'] = animations[item]; });
			
			// Function to generate the CSS for the keyframes
			var generateKeyframeCSS = function(ev, position) {
				
				// Generate keyframes CSS
				keyframe_css = '@' + this.prefix + 'keyframes jBox-animation-' + this.options.animation[ev] + '-' + ev + (position ? '-' + position : '') + ' {';
				jQuery.each(animations[this.options.animation[ev]].css, function(index, item) {
					var translate = position ? item[1].replace('%XY', this._getXY(position).toUpperCase()) : item[1];
					animations[this.options.animation[ev]].positions && (translate = translate.replace('%V', animations[this.options.animation[ev]].positions[position][item[0]]));
					keyframe_css += item[0] + ' {' + this.prefix + 'transform:' + translate + ';}';
				}.bind(this));
				keyframe_css += '}';
				
				// Generate class CSS
				keyframe_css += '.jBox-animation-' + this.options.animation[ev] + '-' + ev + (position ? '-' + position : '') + ' {';
				keyframe_css += this.prefix + 'animation-duration: ' + animations[this.options.animation[ev]].duration + 'ms;';
				keyframe_css += this.prefix + 'animation-name: jBox-animation-' + this.options.animation[ev] + '-' + ev + (position ? '-' + position : '') + ';';
				keyframe_css += animations[this.options.animation[ev]].timing ? (this.prefix + 'animation-timing-function: ' + animations[this.options.animation[ev]].timing + ';') : '';
				keyframe_css += '}';
				
				return keyframe_css;
			}.bind(this);
			
			// Generate css for each event and positions
			var css = '';
			jQuery.each(['open', 'close'], function(index, ev) {
				// No CSS needed for closing with no fade
				if (!this.options.animation[ev] || !animations[this.options.animation[ev]] || (ev == 'close' && !this.options.fade)) return '';
				
				// Generate CSS
				animations[this.options.animation[ev]].positions ?
					jQuery.each(['top', 'right', 'bottom', 'left'], function(index2, position) { css += generateKeyframeCSS(ev, position); }) :
					css += generateKeyframeCSS(ev);
			}.bind(this));
			
			jQuery('<style/>').append(css).appendTo(jQuery('head'));
		};
		
		// Block body clicks for 10ms to prevent extra event triggering
		this._blockBodyClick = function() {
			this.blockBodyClick = true;
			setTimeout(function() { this.blockBodyClick = false; }.bind(this), 10);
		};
		
		// Add css for animations
		this.options.animation && this._generateCSS();
		
		// Animations
		this._animate = function(ev) {
			if (this.IE8) return;
			ev || (ev = this.isOpen ? 'open' : 'close');
			
			// Don't animate when closing with no fade duration
			if (!this.options.fade && ev == 'close') return null;
			
			// Get the current position, use opposite if jBox is flipped
			var animationDirection = (this.options.animation[ev + 'Direction'] || ((this.align != 'center') ? this.align : this.options.attributes.x));
			this.flipped && this._getXY(animationDirection) == (this._getXY(this.align)) && (animationDirection = this._getOpp(animationDirection));
			
			// Add event and position classes
			var classnames = 'jBox-animation-' + this.options.animation[ev] + '-' + ev + ' jBox-animation-' + this.options.animation[ev] + '-' + ev + '-' + animationDirection;
			this.wrapper.addClass(classnames);
			
			// Get duration of animation
			var animationDuration = parseFloat(this.wrapper.css(this.prefix + 'animation-duration')) * 1000;
			ev == 'close' && (animationDuration = Math.min(animationDuration, this.options.fade));
			
			// Remove animation classes when animation is finished
			setTimeout(function() { this.wrapper.removeClass(classnames); }.bind(this), animationDuration);
		};
		
		// Abort animation
		this._abortAnimation = function() {
			if (this.IE8) return;
			
			// Remove all animation classes
			var prefix = 'jBox-animation';
			var classes = this.wrapper.attr('class').split(' ').filter(function(c) {
				return c.lastIndexOf(prefix, 0) !== 0;
			});
			this.wrapper.attr('class', classes.join(' '));
		};
		
		// Adjust position
		this._adjustPosition = function() {
			if (!this.options.adjustPosition) return null;
			
			// Reset cached pointer position
			if (this.positionAdjusted) {
				this.wrapper.css(this.pos);
				this.pointer && this.wrapper.css('padding', 0).css('padding-' + this._getOpp(this.outside), this.pointer.dimensions[this._getXY(this.outside)]).removeClass('jBox-pointerPosition-' + this._getOpp(this.pointer.position)).addClass('jBox-pointerPosition-' + this.pointer.position);
				this.pointer && this.pointer.element.attr('class', 'jBox-pointer jBox-pointer-' + this._getOpp(this.outside)).css(this.pointer.margin);
				this.positionAdjusted = false;
				this.flipped = false;
			}
			
			// Get the window dimensions
			var win = jQuery(window);
			var windowDimensions = {
				x: win.width(),
				y: win.height(),
				top: (this.options.fixed && this.target.data('jBox-fixed') ? 0 : win.scrollTop()),
				left: (this.options.fixed && this.target.data('jBox-fixed') ? 0 : win.scrollLeft())
			};
			windowDimensions.bottom = windowDimensions.top + windowDimensions.y;
			windowDimensions.right = windowDimensions.left + windowDimensions.x;
			
			// Find out where the jBox is out of view area
			var outYT = (windowDimensions.top > this.pos.top - (this.options.adjustDistance.top || 0)),
				outXR = (windowDimensions.right < this.pos.left + this.dimensions.x + (this.options.adjustDistance.right || 0)),
				outYB = (windowDimensions.bottom < this.pos.top + this.dimensions.y + (this.options.adjustDistance.bottom || 0)),
				outXL = (windowDimensions.left > this.pos.left - (this.options.adjustDistance.left || 0)),
				outX = outXL ? 'left' : (outXR ? 'right' : null),
				outY = outYT ? 'top' : (outYB ? 'bottom' : null),
				out = outX || outY;
				
			// Stop here if jBox is not out of view area
			if (!out) return;
			
			// Flip jBox
			if (this.options.adjustPosition != 'move' && (outX == this.outside || outY == this.outside)) {
				
				this.target == 'mouse' && (this.outside = 'right');
				
				// Check if enough space is availible on opposite position
				if (((this.outside == 'top' || this.outside == 'left') ?
					(windowDimensions[this._getXY(this.outside)] - (this.targetDimensions[this._getTL(this.outside)] - windowDimensions[this._getTL(this.outside)]) - this.targetDimensions[this._getXY(this.outside)]) + this.options.offset[this._getXY(this.outside)] :
					(this.targetDimensions[this._getTL(this.outside)] - windowDimensions[this._getTL(this.outside)]) - this.options.offset[this._getXY(this.outside)]
					) > this.dimensions[this._getXY(this.outside)] + parseInt(this.options.adjustDistance[this._getOpp(this.outside)])) {

					// Adjust wrapper and pointer
					this.wrapper.css(this._getTL(this.outside), this.pos[this._getTL(this.outside)] + ((this.dimensions[this._getXY(this.outside)] + (this.options.offset[this._getXY(this.outside)] * (this.outside == 'top' || this.outside == 'left' ? -2 : 2)) + this.targetDimensions[this._getXY(this.outside)]) * (this.outside == 'top' || this.outside == 'left' ? 1 : -1)));
					this.pointer && this.wrapper.removeClass('jBox-pointerPosition-' + this.pointer.position).addClass('jBox-pointerPosition-' + this._getOpp(this.pointer.position)).css('padding', 0).css('padding-' + this.outside, this.pointer.dimensions[this._getXY(this.outside)]);
					this.pointer && this.pointer.element.attr('class', 'jBox-pointer jBox-pointer-' + this.outside);
					this.positionAdjusted = true;
					this.flipped = true;
				}
			}
			
			// Move jBox (only possible with pointer)
			var outMove = (this._getXY(this.outside) == 'x') ? outY : outX;
			
			if (this.pointer && this.options.adjustPosition != 'flip' && this._getXY(outMove) == this._getOpp(this._getXY(this.outside))) {
				
				// Get the maximum space we have availible to adjust
				if (this.pointer.align == 'center') {
					var spaceAvail = (this.dimensions[this._getXY(outMove)] / 2) - (this.pointer.dimensions[this._getOpp(this.pointer.xy)] / 2) - (parseInt(this.pointer.element.css('margin-' + this.pointer.alignAttribute)) * (outMove != this._getTL(outMove) ? -1 : 1));
				} else {
					var spaceAvail = (outMove == this.pointer.alignAttribute) ?
						parseInt(this.pointer.element.css('margin-' + this.pointer.alignAttribute)) :
						this.dimensions[this._getXY(outMove)] - parseInt(this.pointer.element.css('margin-' + this.pointer.alignAttribute)) - this.pointer.dimensions[this._getXY(outMove)];
				}
				
				// Get the overlapping space
				spaceDiff = (outMove == this._getTL(outMove)) ?
					windowDimensions[this._getTL(outMove)] - this.pos[this._getTL(outMove)] + this.options.adjustDistance[outMove] :
					(windowDimensions[this._getOpp(this._getTL(outMove))] - this.pos[this._getTL(outMove)] - this.options.adjustDistance[outMove] - this.dimensions[this._getXY(outMove)]) * -1;
					
				// Add overlapping space on left or top window edge
				if (outMove == this._getOpp(this._getTL(outMove)) && this.pos[this._getTL(outMove)] - spaceDiff < windowDimensions[this._getTL(outMove)] + this.options.adjustDistance[this._getTL(outMove)]) {
					spaceDiff -= windowDimensions[this._getTL(outMove)] + this.options.adjustDistance[this._getTL(outMove)] - (this.pos[this._getTL(outMove)] - spaceDiff);
				}
				
				// Only adjust the maximum availible
				spaceDiff = Math.min(spaceDiff, spaceAvail);
				
				// Move jBox
				if (spaceDiff <= spaceAvail && spaceDiff > 0) {
					this.pointer.element.css('margin-' + this.pointer.alignAttribute, parseInt(this.pointer.element.css('margin-' + this.pointer.alignAttribute)) - (spaceDiff * (outMove != this.pointer.alignAttribute ? -1 : 1)));
					this.wrapper.css(this._getTL(outMove), this.pos[this._getTL(outMove)] + (spaceDiff * (outMove != this._getTL(outMove) ? -1 : 1)));
					this.positionAdjusted = true;
				}
			}
		};
		
		// Fire onInit event
		this._fireEvent('onInit');
		
		return this;
	};

	// Attach jBox to elements
	jBox.prototype.attach = function(elements, trigger) {
		elements || (elements = jQuery(this.options.attach.selector || this.options.attach));
		trigger || (trigger = this.options.trigger);
		
		elements && elements.length && jQuery.each(elements, function(index, el) {
			el = jQuery(el);
			if (!el.data('jBox-attached-' + this.id)) {
				
				// Remove title attribute and store content on element
				(this.options.getContent == 'title' && el.attr('title') != undefined) && el.data('jBox-getContent', el.attr('title')).removeAttr('title');
				
				// Add Element to collection
				this.attachedElements || (this.attachedElements = []);
				this.attachedElements.push(el[0]);
				
				// Add click or mouseenter event, click events can prevent default as well
				el.on(trigger + '.jBox-attach-' + this.id, function(ev) {
					// Clear timer
					this.timer && clearTimeout(this.timer);
					
					// Block opening when jbox is open and the source element is triggering
					if (trigger == 'mouseenter' && this.isOpen && this.source[0] == el[0])
						return;
					
					// Only close jBox if you click the current target element, otherwise open at new target
					if (this.isOpen && this.source && this.source[0] != el[0]) var forceOpen = true;
					
					// Set new source element
					this.source = el;
					
					// Set new target
					!this.options.target && (this.target = el);
					
					// Prevent default action on click
					trigger == 'click' && this.options.preventDefault && ev.preventDefault();
					
					// Toggle or open jBox
					this[trigger == 'click' && !forceOpen ? 'toggle' : 'open']();
				}.bind(this));
				
				// Add close event for trigger event mouseenter
				(this.options.trigger == 'mouseenter') && el.on('mouseleave', function(ev) {
					// If we have set closeOnMouseleave, do not close jBox when leaving attached element and mouse is over jBox
					if(!this.options.closeOnMouseleave || !(ev.relatedTarget == this.wrapper[0] || jQuery(ev.relatedTarget).parents('#' + this.id).length)) this.close();
				}.bind(this));
				
				el.data('jBox-attached-' + this.id, trigger);
				
				// Fire onAttach event
				this._fireEvent('onAttach', el);
			}
		}.bind(this));
		
		return this;
	};

	// Detach jBox from elements
	jBox.prototype.detach = function(elements) {
		elements || (elements = this.attachedElements || []);
		
		elements && elements.length && jQuery.each(elements, function(index, el) {
			el = jQuery(el);
			// Remove events
			if (el.data('jBox-attached-' + this.id)) {
				el.off(el.data('jBox-attached-' + this.id) + '.jBox-attach-' + this.id);
				el.data('jBox-attached-' + this.id, null);
			}
			// Remove element from collection
			this.attachedElements = jQuery.grep(this.attachedElements, function(value) {
				return value != el[0];
			});
		}.bind(this));
		
		return this;
	};

	// Set title
	jBox.prototype.setTitle = function(title, ignore_positioning) {
		!this.wrapper && this._create();
		
		var wrapperHeight = this.wrapper.height(), wrapperWidth = this.wrapper.width();
		if (title == null || title == undefined) return this;
		
		if (!this.title) {
			this.titleContainer = jQuery('<div/>', {'class': 'jBox-title'});
			this.title = jQuery('<div/>').appendTo(this.titleContainer);
			this.wrapper.addClass('jBox-hasTitle');
			if (this.options.closeButton == 'title' || (this.options.closeButton === true && !this.options.overlay)) {
				this.wrapper.addClass('jBox-closeButton-title');
				this.closeButton.appendTo(this.titleContainer);
			}
			this.titleContainer.insertBefore(this.content);
		}
		this.title.html(title);
		
		// Reposition if dimensions changed
		!ignore_positioning && this.options.repositionOnContent && (wrapperHeight != this.wrapper.height() || wrapperWidth != this.wrapper.width()) && this.position();
		
		return this;
	};

	// Set content
	jBox.prototype.setContent = function(content, ignore_positioning) {
		if (content == null) return this;
		
		// Create jBox if no wrapper found
		!this.wrapper && this._create();
		
		// Get the width and height of wrapper, only if they change we need to reposition
		var wrapperHeight = this.wrapper.height(), wrapperWidth = this.wrapper.width();
		
		// Get the width and height of body, if they change with new content, adjust accordingly (happens when a hidden scrollbar changes body dimensions)
		var bodyHeight = jQuery('body').height(), bodyWidth = jQuery('body').width();
		
		// Extract all appended containers to body
		this.content.children('[data-jbox-content-appended]').appendTo('body').css({display: 'none'});
		
		// Set the new content
		switch (jQuery.type(content)) {
			case 'string': this.content.html(content); break;
			case 'object': this.content.html(''); content.attr('data-jbox-content-appended', 1).appendTo(this.content).css({display: 'block'}); break;
	 	}
	 	
		// Calculate the difference to before the content was set
		var adjustOffset = {
			x: bodyWidth - jQuery('body').width(),
			y: bodyHeight - jQuery('body').height()
		};
		
		// Reposition if dimensions changed
		!ignore_positioning && this.options.repositionOnContent && (wrapperHeight != this.wrapper.height() || wrapperWidth != this.wrapper.width()) && this.position({adjustOffset: adjustOffset});
		
		return this;
	};

	// Set new dimensions
	jBox.prototype.setDimensions = function(type, val, pos) {
		
		// Create jBox if no wrapper found
		!this.wrapper && this._create();
		
		// Default value is 'auto'
		val == undefined && (val == 'auto');
		
		// Set CSS of content
		this.content.css(type, val);
		
		// Reposition by default
		(pos == undefined || pos) && this.position();
	};

	// Set width or height
	jBox.prototype.setWidth = function(val, pos) { this.setDimensions('width', val, pos); };
	jBox.prototype.setHeight = function(val, pos) { this.setDimensions('height', val, pos); };

	// Position jBox
	jBox.prototype.position = function(options, keep_position) {
		options || (options = {});
		
		// Get target
		this.target = options.target || this.target || this.options.target || jQuery(window);
		
		// Move outside of scroll area to make sure dimensions are calculated correctly
		!keep_position && this.wrapper.css({top: -12000, left: -12000});
		
		// Cache total current dimensions of jBox
		this.dimensions = {
			x: this.wrapper.outerWidth(),
			y: this.wrapper.outerHeight()
		};
		
		// Mousemove can't be positioned
		if (this.target == 'mouse') return;
		
		// Set percent and margin for centered inside
		if (this.options.position.x == 'center' && this.options.position.y == 'center') {
			this.wrapper.css({left: '50%', top: '50%', marginLeft: (this.dimensions.x * -0.5 + this.options.offset.x), marginTop: (this.dimensions.y * -0.5 + this.options.offset.y)});
			return this;
		}
		
		// Total current dimensions of target element
		var targetOffset = this.target.offset();
		
		// Add fixed data to target
		!this.target.data('jBox-fixed') && this.target.data('jBox-fixed', (this.target[0] != jQuery(window)[0] && (this.target.css('position') == 'fixed' || this.target.parents().filter(function() { return jQuery(this).css('position') == 'fixed'; }).length > 0)) ? 'fixed' : 'static');
		
		// When the target is fixed and jBox is fixed, remove scroll offset
		if (this.target.data('jBox-fixed') == 'fixed' && this.options.fixed) {
			targetOffset.top = targetOffset.top - jQuery(window).scrollTop();
			targetOffset.left = targetOffset.left - jQuery(window).scrollLeft();
		}
		
		// Store target dimensions
		this.targetDimensions = {
			x: this.target.outerWidth(),
			y: this.target.outerHeight(),
			top: (targetOffset ? targetOffset.top : 0),
			left: (targetOffset ? targetOffset.left : 0)
		};
		this.pos = {};
		
		// Calculate positions
		var setPosition = function(p) {
			
			// Set number positions
			if (jQuery.inArray(this.options.position[p], ['top', 'right', 'bottom', 'left', 'center']) == -1) {
				this.pos[this.options.attributes[p]] = this.options.position[p];
				return;
			}
			
			// We have a target, so use 'left' or 'top' as attributes
			var a = this.options.attributes[p] = (p == 'x' ? 'left' : 'top');
			
			// Start at target position
			this.pos[a] = this.targetDimensions[a];
			
			// Set centered position
			if (this.options.position[p] == 'center') {
				this.pos[a] += Math.ceil((this.targetDimensions[p] - this.dimensions[p]) / 2);
				return;
			}
			
			// Move inside
			(a != this.options.position[p]) && (this.pos[a] += this.targetDimensions[p] - this.dimensions[p]);
			
			// Move outside
			(this.options.outside == p || this.options.outside == 'xy') && (this.pos[a] += this.dimensions[p] * (a != this.options.position[p] ? 1 : -1));
			
		}.bind(this);
		
		// Set position including offset
		setPosition('x');
		setPosition('y');
		
		// Adjust position depending on pointer align
		if (this.options.pointer && jQuery.type(this.options.position.x) != 'number' && jQuery.type(this.options.position.y) != 'number') {
			
			var adjustWrapper = 0;
			
			// Where is the pointer aligned? Add or substract accordingly
			switch (this.pointer.align) {
				case 'center':
				if (this.options.position[this._getOpp(this.options.outside)] != 'center') {
					adjustWrapper += (this.dimensions[this._getOpp(this.options.outside)] / 2);
				}
				break;
				default:
				switch (this.options.position[this._getOpp(this.options.outside)]) {
					case 'center':
						adjustWrapper += ((this.dimensions[this._getOpp(this.options.outside)] / 2) - (this.pointer.dimensions[this._getOpp(this.options.outside)] / 2)) * (this.pointer.align == this._getTL(this.pointer.align) ? 1 : -1);
					break;
					default:
						adjustWrapper += (this.pointer.align != this.options.position[this._getOpp(this.options.outside)]) ?
							
						// If pointer align is different to position align
						(this.dimensions[this._getOpp(this.options.outside)] * (jQuery.inArray(this.pointer.align, ['top', 'left']) !== -1 ? 1 : -1)) + ((this.pointer.dimensions[this._getOpp(this.options.outside)] / 2) * (jQuery.inArray(this.pointer.align, ['top', 'left']) !== -1 ? -1 : 1)) :
							
						// If pointer align is same as position align
						(this.pointer.dimensions[this._getOpp(this.options.outside)] / 2) * (jQuery.inArray(this.pointer.align, ['top', 'left']) !== -1 ? 1 : -1);
					break;
				}
				break;
			}
			adjustWrapper *= (this.options.position[this._getOpp(this.options.outside)] == this.pointer.alignAttribute ? -1 : 1);
			adjustWrapper += this.pointer.offset * (this.pointer.align == this._getOpp(this._getTL(this.pointer.align)) ? 1 : -1);
			
			this.pos[this._getTL(this._getOpp(this.pointer.xy))] += adjustWrapper;
		}
		
		// Add adjustments
		options.adjustOffset && options.adjustOffset.x && (this.pos[this.options.attributes.x] += parseInt(options.adjustOffset.x) * (this.options.attributes.x == 'left' ? 1 : -1));
		options.adjustOffset && options.adjustOffset.y && (this.pos[this.options.attributes.y] += parseInt(options.adjustOffset.y) * (this.options.attributes.y == 'top' ? 1 : -1));
		
		// Add final offset
		this.pos[this.options.attributes.x] += this.options.offset.x;
		this.pos[this.options.attributes.y] += this.options.offset.y;
		
		// Set CSS
		this.wrapper.css(this.pos);
		
		/*
			TODO Better solution
				 Check this for responsive design
				 
		// If by positioning dimensions changed, position again without recalculating
		if (!keep_position && this.wrapper.outerWidth() != this.dimensions.x || this.wrapper.outerHeight() != this.dimensions.y) {
			return this.position(options, true);
		}
		
		*/
		
		// Adjust position
		this._adjustPosition();
		
		return this;
	};

	// Open jBox
	jBox.prototype.open = function(options) {
		options || (options = {});
		
		// Abort if jBox was destroyed
		if (this.isDestroyed) return false;
		
		// Construct jBox if not already constructed
		!this.wrapper && this._create();
		
		// Abort any opening or closing timer
		this.timer && clearTimeout(this.timer);
		
		// Block body click for 10ms, so jBox can open on attached elements while closeOnClick = 'body'
		this._blockBodyClick();
		
		// Block opening
		if (this.isDisabled) return this;
		
		// Opening function
		var open = function() {
			
			// Set title from source element
			this.source && this.options.getTitle && (this.source.attr(this.options.getTitle) && this.setTitle(this.source.attr(this.options.getTitle)), true);
			
			// Set content from source element
			this.source && this.options.getContent && (this.source.data('jBox-getContent') ? this.setContent(this.source.data('jBox-getContent'), true) : (this.source.attr(this.options.getContent) ? this.setContent(this.source.attr(this.options.getContent), true) : null));
			
			// Fire onOpen event
			this._fireEvent('onOpen');
			
			// Get content from ajax
			((this.options.ajax && this.options.ajax.url && (!this.ajaxLoaded || this.options.ajax.reload)) || (options.ajax && options.ajax.url)) && this.ajax(options.ajax || null);
			
			// Set position
			(!this.positionedOnOpen || this.options.repositionOnOpen) && this.position({target: options.target}) && (this.positionedOnOpen = true);
			
			// Abort closing
			this.isClosing && this._abortAnimation();
			
			// Open functions to call when jBox is closed
			if (!this.isOpen) {
				
				// jBox is open now
				this.isOpen = true;
				
				// Attach events
				this._attachEvents();
				
				// Block scrolling
				this.options.blockScroll && jQuery('body').addClass('jBox-blockScroll-' + this.id);
				
				// Add overlay
				this.options.overlay && this._addOverlay();
				
				// Only animate if jBox is compleately closed
				this.options.animation && !this.isClosing && this._animate('open');
				
				// Fading animation or show immediately
				if (this.options.fade) {
					this.wrapper.stop().animate({opacity: 1}, {
						queue: false,
						duration: this.options.fade,
						start: function() {
							this.isOpening = true;
							this.wrapper.css({display: 'block'});
						}.bind(this),
						always: function() {
							this.isOpening = false;
						}.bind(this)
					});
				} else {
					this.wrapper.css({display: 'block', opacity: 1});
				}
			}
		}.bind(this);
		
		// Open jBox
		this.options.delayOpen && !this.isOpen && !this.isClosing && !options.ignoreDelay ? (this.timer = setTimeout(open, this.options.delayOpen)) : open();
		
		return this;
	};

	// Close jBox
	jBox.prototype.close = function(options) {
		options || (options = {});
		
		// Abort if jBox was destroyed
		if (this.isDestroyed) return false;
		
		// Abort opening
		this.timer && clearTimeout(this.timer);
		
		// Block body click for 10ms, so jBox can open on attached elements while closeOnClock = 'body' is true
		this._blockBodyClick();
		
		// Block closing
		if (this.isDisabled) return this;
		
		// Close function
		var close = function() {
			
			// Fire onClose event
			this._fireEvent('onClose');
			
			// Only close if jBox is open
			if (this.isOpen) {
				
				// jBox is not open anymore
				this.isOpen = false;
				
				// Detach events
				this._detachEvents();
				
				// Unblock scrolling
				this.options.blockScroll && jQuery('body').removeClass('jBox-blockScroll-' + this.id);
				
				// Remove overlay
				this.options.overlay && this._removeOverlay();
				
				// Only animate if jBox is compleately closed
				this.options.animation && !this.isOpening && this._animate('close');
				
				// Fading animation or show immediately
				if (this.options.fade) {
					this.wrapper.stop().animate({opacity: 0}, {
						queue: false,
						duration: this.options.fade,
						start: function() {
							this.isClosing = true;
						}.bind(this),
						complete: function() {
							this.wrapper.css({display: 'none'});
							this.options.onCloseComplete && (this.options.onCloseComplete.bind(this))();
							this.options._onCloseComplete && (this.options._onCloseComplete.bind(this))();
						}.bind(this),
						always: function() {
							this.isClosing = false;
						}.bind(this)
					});
				} else {
					this.wrapper.css({display: 'none', opacity: 0});
					this.options._onCloseComplete && (this.options._onCloseComplete.bind(this))();
				}
			}
		}.bind(this);
		
		// Close jBox
		options.ignoreDelay ? close() : (this.timer = setTimeout(close, Math.max(this.options.delayClose, 10)));
		
		return this;
	};

	// Open or close jBox
	jBox.prototype.toggle = function(options) {
		this[this.isOpen ? 'close' : 'open'](options);
		return this;
	};

	// Block opening and closing
	jBox.prototype.disable = function() {
		this.isDisabled = true;
		return this;
	};

	// Unblock opening and closing
	jBox.prototype.enable = function() {
		this.isDisabled = false;
		return this;
	};

	// Get content from ajax
	jBox.prototype.ajax = function(options) {
		options || (options = {});
		
		// Add data from source element if none set in options
		(this.options.ajax.getData && !options.data && this.source && this.source.attr(this.options.ajax.getData) != undefined) && (options.data = this.source.attr(this.options.ajax.getData) || '');
		
		// Clone the system options
		var sysOptions = jQuery.extend(true, {}, this.options.ajax);
		
		// Abort running ajax call
		this.ajaxRequest && this.ajaxRequest.abort();
		
		// Extract events
		var beforeSend = options.beforeSend || sysOptions.beforeSend || function () {};
		var complete = options.complete || sysOptions.complete || function () {};
		
		// Merge options
		var userOptions = jQuery.extend(true, sysOptions, options);
		
		// Set new beforeSend event
		userOptions.beforeSend = function () {
			
			// Add loading spinner
			if (userOptions.spinner) {
				this.wrapper.addClass('jBox-loading');
				this.spinner = jQuery(userOptions.spinner !== true ? userOptions.spinner : '<div class="jBox-spinner"></div>').appendTo(this.container);
			}
			
			(beforeSend.bind(this))();
		}.bind(this);
		
		// Set new complete event
		userOptions.complete = function (response) {
			
			// Remove spinner
			this.wrapper.removeClass('jBox-loading');
			this.spinner && this.spinner.remove();
			
			// Set new content
			userOptions.setContent && this.setContent(response.responseText);
			
			this.ajaxLoaded = true;
			
			(complete.bind(this))(response);
		}.bind(this);
		
		// Send new ajax request
		this.ajaxRequest = jQuery.ajax(userOptions);
		
		return this;
	};

	// Play an audio file
	jBox.prototype.audio = function(options) {
		options || (options = {});
		jBox._audio || (jBox._audio = {});
		
		// URL required, no IE8 support
		if (!options.url || this.IE8) return this;
		
		// Create audio if it doesn't exist
		if (!jBox._audio[options.url]) {
			var audio = jQuery('<audio/>');
			jQuery('<source/>', {src: options.url + '.mp3'}).appendTo(audio);
			jQuery('<source/>', {src: options.url + '.ogg'}).appendTo(audio);
			jBox._audio[options.url] = audio[0];
		}
		
		// Set volume and play audio
		jBox._audio[options.url].volume = Math.min((options.volume != undefined ? options.volume : (this.options.volume != undefined ? this.options.volume : 100) / 100), 1);
		jBox._audio[options.url].pause();
		try { jBox._audio[options.url].currentTime = 0; } catch (e) {}
		jBox._audio[options.url].play();
		
		return this;
	};

	// Destroy jBox and remove it from DOM
	// TODO: If no other jBox needs an overlay remove it as well
	jBox.prototype.destroy = function() {
		this.detach().close({ignoreDelay: true});
		this.wrapper && this.wrapper.remove();
		this.isDestroyed = true;
		return this;
	};

	// TODO: Find an option to preload audio files

	// Get a unique ID for jBoxes
	jBox._getUniqueID = (function () {
		var i = 1;
		return function () {
			return i++;
		};
	}());

	// Make jBox usable with jQuery selectors
	jQuery.fn.jBox = function(type, options) {
		type || (type = {});
		options || (options = {});
		return new jBox(type, jQuery.extend(options, {attach: this}));
	};

	// Add the .bind() function for IE 8 support
	if (!Function.prototype.bind) {
		Function.prototype.bind = function (oThis) {
			var aArgs = Array.prototype.slice.call(arguments, 1),
				fToBind = this,
				fNOP = function () {},
				fBound = function () { return fToBind.apply(this instanceof fNOP && oThis ? this : oThis, aArgs.concat(Array.prototype.slice.call(arguments))); };
			fNOP.prototype = this.prototype;
			fBound.prototype = new fNOP();
			return fBound;
		};
	}

	return jBox;
}));
