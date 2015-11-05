jBox
====

jBox is a powerful and flexible jQuery plugin, taking care of all your modal windows, tooltips, notices and more.

Demo: http://stephanwagner.me/jBox

Docs: http://stephanwagner.me/jBox/documentation


Tooltips
--------

You can use jQuery selectors to add tooltips to elements:

	$('.tooltip').jBox('Tooltip');

Now elements with class="tooltip" will open tooltips:

	<span class="tooltip" title="My first tooltip">Hover me!</span>
	<span class="tooltip" title="My second tooltip">Hover me!</span>


Modal windows
-------------

You can set up modal windows the same way as tooltips.
But most of times you'd want more variety, like a title or HTML content:

	new jBox('Modal', {
		width: 300,
		height: 200,
		attach: $('#myModal'),
		title: 'My Modal Window',
		content: '<i>Hello there!</i>'
	});
	
	<div id="myModal">Click me to open a modal window!</div>



Confirm windows
---------------

Confirm windows are modal windows which requires the user to confirm a click action on an element.
Give an element the attribute data-confirm to attach it:

	new jBox('Confirm', {
		confirmButton: 'Do it!',
		cancelButton: 'Nope'
	});
	
	<div onclick="doit()" data-confirm="Do you really want to do this?">Click me!</div>
	<a href="http://stephanwagner.me" data-confirm="Do you really want to leave this page?">Click me!</a>


Notices
-------

A notice will open automatically and destroy itself after some time:

	new jBox('Notice', {
		content: 'Hurray! A notice!'
	});


Images
------

To create image modal windows you only need following few lines:

	new jBox('Image');
	
	<a href="/image-large.jpg" data-jbox-image="gallery1" title="My image"><img src="/image.jpg" alt=""></a>


Learn more
----------

These few examples are very basic.
The jBox library is quite powerful and offers a vast variety of options to customize appearance and behavior.
Learn more in the documentation: http://stephanwagner.me/jBox/documentation
