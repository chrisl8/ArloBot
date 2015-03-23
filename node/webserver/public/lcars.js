/*
LCARS Buttons taken from LCARS Framework jQuery Plugin v2.0 (FRAGMENT!)
Requires Jquery 1.4.*
@author: Josh Messer
@date: 1.26.2011
The original code is here: http://lcars.ws/lcarsdist/ (dead link)
The original code is now here: http://www.siriustrader.com/LCARS/jquery.lcars-master/
This is only a fragment of the original code!
*/
// This is the format to extend jQuery with a new function
// http://stackoverflow.com/questions/2937227/what-does-function-jquery-mean

// for jsLint, which I did use to tidy this up a bit and find/fix errors I made when customizing and trimming.
/*jslint nomen: true */
/*global jQuery*/

// http://www.lcars47.com/p/lcars-101.html
// Last column, Star Trek: Nemesis colors
// Modeling color after this picture from  Star Trek: Nemesis:
// http://upload.wikimedia.org/wikipedia/en/4/4b/Enterprise-E_LCARS.jpg
// The LCARS jQuery system uses ten colors, but there are only 8 in the pallet
// Also, I'm resorting these to match the list
// I am also using #FF9900 for some text and buttons as seen in the above image,
// Or at least I think I see that color, at least it looks good to me.
function lcars_colors(input_color) {
    'use strict'; // http://www.w3schools.com/js/js_strict.asp
    var lcars_color = [];
    lcars_color.white = "#CCDDFF";
    lcars_color.lightBlue = "#5599FF";
    lcars_color.lightTan = "#3366FF";
    // Less Light Blue
    lcars_color.pink = "#cc6699";
    // Extra Color, from TNG later series (2nd column)
    lcars_color.lightRed = "#cc6666";
    // Extra Color, from TNG later series (2nd column)
    lcars_color.blue = "#0011EE";
    lcars_color.purple = "#000088";
    lcars_color.tan = "#BBAA55";
    //lcars_color.orange = "#BB4411";
    lcars_color.orange = "#FF9900";
    lcars_color.red = "#882211";
    var color = lcars_color[input_color];
    if (color === undefined) {
        return input_color;
        // No need for } else { here because if this happens, the function ends there!
    }
    return color;
}

(function ($) {
    'use strict'; // http://www.w3schools.com/js/js_strict.asp
    $.fn.lcarsButton = function (options) {// set defaults
        var defaults = {
            rounded : 'both', // accepts both, left, right, none
            extended : false, // this is true or false
            color : 'orange',
            subTitle : {// The sub title for your button
                direction : 'none', // left or right
                text : '' // the text for the sub title
            },
            blank : 'none' // blank button? left / right / none
        };
        var theOptions = $.extend(defaults, options);

        // here we get the class for the rounded button
        var button_rounded = [];
        button_rounded.both = "RR";
        button_rounded.right = "SR";
        button_rounded.left = "RS";
        button_rounded.none = "";
        theOptions.rounded = button_rounded[theOptions.rounded];

        var _extend = '';
        if (theOptions.extended === true) {
            _extend = 'L';
        }

        var subtitle_dir = [];
        subtitle_dir.left = 'B_titleL';
        subtitle_dir.right = 'B_titleR';
        subtitle_dir.none = undefined;
        theOptions.subTitle.direction = subtitle_dir[theOptions.subTitle.direction];
        if (theOptions.subTitle.text === undefined) {
            theOptions.subTitle.text = '';
        }

        var blank_dir = [];
        blank_dir.none = undefined;
        blank_dir.left = 'B_blankL';
        blank_dir.right = 'B_blankR';
        theOptions.blank = blank_dir[theOptions.blank];

        this.each(function () {
            var _this = $(this);
            if (!_this.hasClass('button')) {
                _this.addClass('button');
            }

            _this.addClass(theOptions.rounded).addClass(_extend).css('background-color', lcars_colors(theOptions.color));

            if (theOptions.subTitle.direction !== '' || theOptions.subTitle.direction !== undefined) {
                _this.prepend('<span class="' + theOptions.subTitle.direction + '" style="color:' + lcars_colors(theOptions.color) + '">' + theOptions.subTitle.text + '</span>');
            }

            if (theOptions.blank !== '' && theOptions.blank !== undefined) {
                if (_this.find('span').length >= 1) {
                    _this.removeClass(theOptions.rounded).find('span').removeClass(theOptions.subTitle.direction).addClass(theOptions.blank).html('');
                }
                if (_this.find('span').length === 0) {
                    _this.removeClass(theOptions.rounded);
                    _this.prepend('<span class="' + theOptions.blank + '"></span>');
                }

                if (theOptions.blank === 'B_blankL') {
                    _this.addClass('RS');
                }
                if (theOptions.blank === 'B_blankR') {
                    _this.addClass('SR');
                }
            }
        });
    };
}(jQuery)); // http://stackoverflow.com/questions/4979252/jslint-error-move-the-invocation-into-the-parens-that-contain-the-function
// This (function () {}()); format indicates a function that is run "in place":
// http://stackoverflow.com/questions/2937227/what-does-function-jquery-mean
