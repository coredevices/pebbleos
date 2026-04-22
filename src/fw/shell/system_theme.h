/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/fonts/fonts.h"
#include "applib/graphics/gtypes.h"
#include "applib/platform.h"
#include "applib/preferred_content_size.h"

//! System Theme Text Style is a font collection used to unify text styles across the system.
//! It contains a variety of different font sizes for use in an application, each meant for a
//! distinct class of use cases. Each font type will resize based on the user's preferences.
//! Consumers should attempt to have a complete mapping of their font types to the system style.

typedef enum TextStyleFont {
  //! Header is for metadata text that gives extra context for the user, such as which application
  //! a notification belongs to, or who sent a message. It is smaller than the body copy when
  //! readable, but always bold.
  TextStyleFont_Header,
#if !defined(CONFIG_RECOVERY_FW)
  //! Title is for prominent text that is usually the title or name of the content, giving context
  //! for the user such as the subject line of an email. It is comparable to the body, but
  //! always bold.
  TextStyleFont_Title,
  //! Body is for body text that can be long stretches of text, such as the body of an email.
  TextStyleFont_Body,
#endif
  //! Subtitle is for subtitle text that should be prominent, such as the title of a section in a
  //! larger content matter. Subtitle can also be used when the title should be large, but not
  //! prominent. It is comparable to the title, but not bold for the Larger theme.
  TextStyleFont_Subtitle,
  //! Caption is for a contextual description text of a subject in a larger content matter.
  //! It is usually smaller than the footer.
  TextStyleFont_Caption,
  //! Footer is for metadata text that the user may be interested in after consuming the main
  //! content, such as age of a notification. It is smaller than the body copy.
  TextStyleFont_Footer,
  //! For titles of menu cells that identify an item of a list.
  TextStyleFont_MenuCellTitle,
  //! For subtitles of menu cells that provide auxiliary information about an item of a list.
  TextStyleFont_MenuCellSubtitle,
#if !defined(CONFIG_RECOVERY_FW)
  //! Time Header Numbers is used specially by Timeline to display content time in extra bold e.g.
  //! 4:06 AM. It is comparable to the title.
  TextStyleFont_TimeHeaderNumbers,
#endif
  //! Time Header Words is used in conjunction with its numbers counterpart for AM PM. The size is
  //! significantly smaller than the numbers counterpart to display AM PM in capitals.
  TextStyleFont_TimeHeaderWords,
  //! Pin Subtitle is for subtitle text where the text box is small used specially by Timeline.
  //! It is smaller than the title and is not bold.
  TextStyleFont_PinSubtitle,
  //! Paragraph Header is for text that describes the content of a body paragraph. The size is
  //! smaller than both Body and Header.
  TextStyleFont_ParagraphHeader,

  TextStyleFontCount
} TextStyleFont;


//! @param font The desired font class to obtain a font key of.
//! @return The font key of the font class using the user's preferred content size.
const char *system_theme_get_font_key(TextStyleFont font);

//! @param size The desired content size.
//! @param font The desired font class to obtain a font key of.
//! @return The font key of the given content size and font class.
const char *system_theme_get_font_key_for_size(PreferredContentSize size, TextStyleFont font);

//! @param font The desired font class to obtain a font of.
//! @return The font of the font class using the user's preferred content size.
GFont system_theme_get_font(TextStyleFont font);

//! @param size The desired content size.
//! @param font The desired font class to obtain a font of.
//! @return The font of the given content size and font class.
GFont system_theme_get_font_for_size(PreferredContentSize size, TextStyleFont font);

//! @param font The desired font class for which to obtain a font for the runtime platform's default
//! size.
//! @return The font of the given font class for the runtime platform's default size.
GFont system_theme_get_font_for_default_size(TextStyleFont font);

//! @param content_size The user's desired content size.
void system_theme_set_content_size(PreferredContentSize content_size);

//! @return The user's preferred content size.
PreferredContentSize system_theme_get_content_size(void);

//! @return The default content size for the current runtime platform
PreferredContentSize system_theme_get_default_content_size_for_runtime_platform(void);

//! @param size The \ref PreferredContentSize to convert from the host to the current runtime
//! platform
//! @return The input \ref PreferredContentSize converted from the host to the current runtime
//! platform
PreferredContentSize system_theme_convert_host_content_size_to_runtime_platform(
    PreferredContentSize size);

//! @return true if the current app context should render in dark mode (system apps only).
bool system_theme_is_dark_mode(void);

//! @return The background color appropriate for the current theme (black in dark mode on color
//!         displays, white otherwise).
GColor system_theme_get_bg_color(void);

//! @return The foreground color appropriate for the current theme (white in dark mode on color
//!         displays, black otherwise).
GColor system_theme_get_fg_color(void);
